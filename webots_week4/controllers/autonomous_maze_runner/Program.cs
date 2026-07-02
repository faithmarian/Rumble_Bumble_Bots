using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;

internal static class Program
{
    private const int TimeStep = 32;
    private const double WheelRadius = 0.016;
    private const double AxleLength = 0.064;
    private const double CellDistance = 0.180;
    private const double TurnAngle = Math.PI / 2.0;

    private const double ForwardSpeed = 1.35;
    private const double TurnSpeed = 0.68;
    private const double MinTurnSpeed = 0.075;
    private const double TurnTolerance = 0.004;
    private const int TurnStableSteps = 5;
    private const int SettleSteps = 8;

    private const double WallSenseDistance = 0.135;
    private const double SideWallUsableRange = 0.130;
    private const double SingleWallUsableRange = 0.090;
    private const double SideWallTarget = 0.063;
    private const double FrontBlockedDistance = 0.043;
    private const double WallCorrectionStart = 0.020;
    private const double WallCorrectionEnd = CellDistance - 0.020;

    private const double ForwardYawKp = 2.0;
    private const double ForwardYawKd = 0.05;
    private const double TurnKp = 1.65;
    private const double TurnKd = 0.10;
    private const double WallCenterKp = 0.45;
    private const double WallCenterKd = 0.015;
    private const double OneWallKp = 0.30;
    private const double OneWallKd = 0.015;
    private const double MaxForwardCorrection = 0.18;
    private const double MaxLidarCorrection = 0.030;

    private const double PreForwardAlignTolerance = 0.006;
    private const double PreForwardAlignMaxSpeed = 0.20;
    private const double PreForwardAlignMinSpeed = 0.065;
    private const int PreForwardAlignStableSteps = 4;
    private const int PreForwardAlignMaxSteps = 60;

    private static readonly int[] RowStep = { -1, 0, 1, 0 };
    private static readonly int[] ColStep = { 0, 1, 0, -1 };

    private static CellInfo[,] maze;
    private static int rows;
    private static int cols;
    private static int startRow;
    private static int startCol;
    private static int goalRow;
    private static int goalCol;
    private static int currentRow;
    private static int currentCol;
    private static Direction heading;
    private static double targetHeading;
    private static int visitedCount;

    private static RobotDevices devices;
    private static double filteredFront = double.NaN;
    private static double filteredLeft = double.NaN;
    private static double filteredRight = double.NaN;

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_robot_init();

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern int wb_robot_step(int duration);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_robot_cleanup();

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr wb_robot_get_custom_data();

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern int wb_robot_get_device(string name);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_motor_set_position(int tag, double position);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_motor_set_velocity(int tag, double velocity);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_position_sensor_enable(int tag, int samplingPeriod);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern double wb_position_sensor_get_value(int tag);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_distance_sensor_enable(int tag, int samplingPeriod);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern double wb_distance_sensor_get_value(int tag);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_inertial_unit_enable(int tag, int samplingPeriod);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr wb_inertial_unit_get_roll_pitch_yaw(int tag);

    private enum Direction
    {
        North = 0,
        East = 1,
        South = 2,
        West = 3
    }

    private enum WallState
    {
        Unknown = 0,
        Open = 1,
        Wall = 2
    }

    private static int Main()
    {
        try
        {
            wb_robot_init();
            devices = InitializeDevices();
            wb_robot_step(TimeStep);

            RunnerConfig config = ParseConfig(PtrToString(wb_robot_get_custom_data()));
            ConfigureMaze(config);

            targetHeading = ReadYaw(devices.Imu);
            Console.WriteLine("Autonomous maze runner started.");
            Console.WriteLine($"Start=({startRow},{startCol},{heading}), Goal=({goalRow},{goalCol}), Maze={rows}x{cols}");

            ScanCurrentCell();
            while (visitedCount < rows * cols)
            {
                List<Direction> pathToFrontier = FindPathToNearestUnvisited();
                if (pathToFrontier.Count == 0)
                {
                    Console.WriteLine($"No more reachable frontier cells. Visited {visitedCount}/{rows * cols}.");
                    break;
                }

                if (!MoveToNeighbor(pathToFrontier[0]))
                {
                    Console.WriteLine("Move failed; edge was marked as a wall and the runner will replan.");
                    ScanCurrentCell();
                    continue;
                }

                ScanCurrentCell();
            }

            Console.WriteLine($"Mapping phase complete: visited {visitedCount}/{rows * cols} = {CompletionPercent():F1}%.");

            List<Direction> returnPath = FindPathToCell(startRow, startCol);
            Console.WriteLine($"Returning to start with {returnPath.Count} cell moves.");
            ExecutePath(returnPath, false);

            List<Direction> goalPath = FindPathToCell(goalRow, goalCol);
            Console.WriteLine($"Shortest known path start -> goal has {goalPath.Count} cell moves.");
            ExecutePath(goalPath, false);

            Stop();
            Console.WriteLine("Autonomous maze runner finished.");

            while (wb_robot_step(TimeStep) != -1)
                Stop();

            wb_robot_cleanup();
            return 0;
        }
        catch (Exception ex)
        {
            Console.WriteLine("autonomous_maze_runner failed: " + ex);
            try
            {
                Stop();
                wb_robot_cleanup();
            }
            catch
            {
            }
            return 1;
        }
    }

    private static RobotDevices InitializeDevices()
    {
        RobotDevices result = new RobotDevices
        {
            LeftMotor = wb_robot_get_device("left wheel motor"),
            RightMotor = wb_robot_get_device("right wheel motor"),
            LeftSensor = wb_robot_get_device("left wheel sensor"),
            RightSensor = wb_robot_get_device("right wheel sensor"),
            Imu = wb_robot_get_device("imu"),
            FrontLidar = wb_robot_get_device("front lidar"),
            LeftLidar = wb_robot_get_device("left lidar"),
            RightLidar = wb_robot_get_device("right lidar")
        };

        wb_motor_set_position(result.LeftMotor, double.PositiveInfinity);
        wb_motor_set_position(result.RightMotor, double.PositiveInfinity);
        wb_motor_set_velocity(result.LeftMotor, 0.0);
        wb_motor_set_velocity(result.RightMotor, 0.0);
        wb_position_sensor_enable(result.LeftSensor, TimeStep);
        wb_position_sensor_enable(result.RightSensor, TimeStep);
        wb_inertial_unit_enable(result.Imu, TimeStep);
        wb_distance_sensor_enable(result.FrontLidar, TimeStep);
        wb_distance_sensor_enable(result.LeftLidar, TimeStep);
        wb_distance_sensor_enable(result.RightLidar, TimeStep);
        return result;
    }

    private static RunnerConfig ParseConfig(string raw)
    {
        RunnerConfig config = new RunnerConfig
        {
            Rows = 9,
            Cols = 9,
            StartRow = 0,
            StartCol = 0,
            GoalRow = 8,
            GoalCol = 8,
            Heading = Direction.East
        };

        raw = raw ?? string.Empty;
        Match grid = Regex.Match(raw, @"GRID\s*=?\s*(\d+)\s*(?:x|,|\s)\s*(\d+)", RegexOptions.IgnoreCase);
        if (grid.Success)
        {
            config.Rows = int.Parse(grid.Groups[1].Value);
            config.Cols = int.Parse(grid.Groups[2].Value);
        }

        Match start = Regex.Match(raw, @"START\s*=?\s*\(?\s*(\d+)\s*(?:,|\s)\s*(\d+)", RegexOptions.IgnoreCase);
        if (start.Success)
        {
            config.StartRow = int.Parse(start.Groups[1].Value);
            config.StartCol = int.Parse(start.Groups[2].Value);
        }

        Match goal = Regex.Match(raw, @"GOAL\s*=?\s*\(?\s*(\d+)\s*(?:,|\s)\s*(\d+)", RegexOptions.IgnoreCase);
        if (goal.Success)
        {
            config.GoalRow = int.Parse(goal.Groups[1].Value);
            config.GoalCol = int.Parse(goal.Groups[2].Value);
        }

        Match headingMatch = Regex.Match(raw, @"HEADING\s*=?\s*(NORTH|EAST|SOUTH|WEST|N|E|S|W)", RegexOptions.IgnoreCase);
        if (headingMatch.Success)
            config.Heading = ParseDirection(headingMatch.Groups[1].Value);

        config.Rows = ClampInt(config.Rows, 2, 30);
        config.Cols = ClampInt(config.Cols, 2, 30);
        config.StartRow = ClampInt(config.StartRow, 0, config.Rows - 1);
        config.StartCol = ClampInt(config.StartCol, 0, config.Cols - 1);
        config.GoalRow = ClampInt(config.GoalRow, 0, config.Rows - 1);
        config.GoalCol = ClampInt(config.GoalCol, 0, config.Cols - 1);
        return config;
    }

    private static void ConfigureMaze(RunnerConfig config)
    {
        rows = config.Rows;
        cols = config.Cols;
        startRow = config.StartRow;
        startCol = config.StartCol;
        goalRow = config.GoalRow;
        goalCol = config.GoalCol;
        currentRow = startRow;
        currentCol = startCol;
        heading = config.Heading;
        visitedCount = 0;

        maze = new CellInfo[rows, cols];
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
                maze[r, c] = new CellInfo();
        }

        for (int c = 0; c < cols; c++)
        {
            SetEdge(0, c, Direction.North, WallState.Wall, true);
            SetEdge(rows - 1, c, Direction.South, WallState.Wall, true);
        }
        for (int r = 0; r < rows; r++)
        {
            SetEdge(r, 0, Direction.West, WallState.Wall, true);
            SetEdge(r, cols - 1, Direction.East, WallState.Wall, true);
        }
    }

    private static void ScanCurrentCell()
    {
        if (!maze[currentRow, currentCol].Visited)
        {
            maze[currentRow, currentCol].Visited = true;
            visitedCount++;
        }

        for (int scan = 0; scan < 4; scan++)
        {
            ResetLidarFilters();
            SensorReadings readings = ReadStableDistances();
            ApplySensorReading(heading, readings.Front);
            ApplySensorReading(LeftOf(heading), readings.Left);
            ApplySensorReading(RightOf(heading), readings.Right);

            Console.WriteLine($"Scan ({currentRow},{currentCol}) heading={heading}: front={readings.Front:F3}, left={readings.Left:F3}, right={readings.Right:F3}");

            Direction unknown = FirstUnknownEdge(currentRow, currentCol);
            if (unknown == DirectionCountSentinel())
                break;

            TurnToDirection(unknown);
        }

        PrintMap();
    }

    private static void ApplySensorReading(Direction direction, double distance)
    {
        int nr = currentRow + RowStep[(int)direction];
        int nc = currentCol + ColStep[(int)direction];
        if (!InBounds(nr, nc))
        {
            SetEdge(currentRow, currentCol, direction, WallState.Wall, true);
            return;
        }

        if (distance > 0.005 && distance < WallSenseDistance)
            SetEdge(currentRow, currentCol, direction, WallState.Wall, false);
        else
            SetEdge(currentRow, currentCol, direction, WallState.Open, false);
    }

    private static Direction FirstUnknownEdge(int row, int col)
    {
        for (int d = 0; d < 4; d++)
        {
            Direction direction = (Direction)d;
            int nr = row + RowStep[d];
            int nc = col + ColStep[d];
            if (!InBounds(nr, nc))
            {
                SetEdge(row, col, direction, WallState.Wall, true);
                continue;
            }
            if (maze[row, col].Walls[d] == WallState.Unknown)
                return direction;
        }
        return DirectionCountSentinel();
    }

    private static List<Direction> FindPathToNearestUnvisited()
    {
        bool[,] seen = new bool[rows, cols];
        int[,] parentRow = NewFilledArray(rows, cols, -1);
        int[,] parentCol = NewFilledArray(rows, cols, -1);
        Direction[,] parentDir = new Direction[rows, cols];
        Queue<CellPoint> queue = new Queue<CellPoint>();
        queue.Enqueue(new CellPoint(currentRow, currentCol));
        seen[currentRow, currentCol] = true;

        while (queue.Count > 0)
        {
            CellPoint cell = queue.Dequeue();
            for (int d = 0; d < 4; d++)
            {
                Direction direction = (Direction)d;
                if (!CanTraverse(cell.Row, cell.Col, direction))
                    continue;

                int nr = cell.Row + RowStep[d];
                int nc = cell.Col + ColStep[d];
                if (!InBounds(nr, nc))
                    continue;

                if (!maze[nr, nc].Visited)
                {
                    List<Direction> path = ReconstructPath(cell.Row, cell.Col, parentRow, parentCol, parentDir);
                    path.Add(direction);
                    return path;
                }

                if (seen[nr, nc])
                    continue;

                seen[nr, nc] = true;
                parentRow[nr, nc] = cell.Row;
                parentCol[nr, nc] = cell.Col;
                parentDir[nr, nc] = direction;
                queue.Enqueue(new CellPoint(nr, nc));
            }
        }

        return new List<Direction>();
    }

    private static List<Direction> FindPathToCell(int targetRow, int targetCol)
    {
        bool[,] seen = new bool[rows, cols];
        int[,] parentRow = NewFilledArray(rows, cols, -1);
        int[,] parentCol = NewFilledArray(rows, cols, -1);
        Direction[,] parentDir = new Direction[rows, cols];
        Queue<CellPoint> queue = new Queue<CellPoint>();
        queue.Enqueue(new CellPoint(currentRow, currentCol));
        seen[currentRow, currentCol] = true;

        while (queue.Count > 0)
        {
            CellPoint cell = queue.Dequeue();
            if (cell.Row == targetRow && cell.Col == targetCol)
                return ReconstructPath(cell.Row, cell.Col, parentRow, parentCol, parentDir);

            for (int d = 0; d < 4; d++)
            {
                Direction direction = (Direction)d;
                if (!CanTraverse(cell.Row, cell.Col, direction))
                    continue;

                int nr = cell.Row + RowStep[d];
                int nc = cell.Col + ColStep[d];
                if (!InBounds(nr, nc) || seen[nr, nc])
                    continue;

                seen[nr, nc] = true;
                parentRow[nr, nc] = cell.Row;
                parentCol[nr, nc] = cell.Col;
                parentDir[nr, nc] = direction;
                queue.Enqueue(new CellPoint(nr, nc));
            }
        }

        Console.WriteLine($"No known path from ({currentRow},{currentCol}) to ({targetRow},{targetCol}).");
        return new List<Direction>();
    }

    private static void ExecutePath(List<Direction> path, bool scanAfterEachMove)
    {
        for (int i = 0; i < path.Count; i++)
        {
            if (!MoveToNeighbor(path[i]))
            {
                Console.WriteLine($"Path execution interrupted at step {i + 1}/{path.Count}.");
                return;
            }
            if (scanAfterEachMove)
                ScanCurrentCell();
        }
    }

    private static bool MoveToNeighbor(Direction direction)
    {
        if (!CanTraverse(currentRow, currentCol, direction))
        {
            Console.WriteLine($"Refusing to move through non-open edge ({currentRow},{currentCol}) -> {direction}.");
            return false;
        }

        TurnToDirection(direction);
        bool ok = MoveForwardOneCell();
        if (!ok)
        {
            SetEdge(currentRow, currentCol, direction, WallState.Wall, true);
            return false;
        }

        SetEdge(currentRow, currentCol, direction, WallState.Open, true);
        currentRow += RowStep[(int)direction];
        currentCol += ColStep[(int)direction];
        Console.WriteLine($"Pose update: ({currentRow},{currentCol}) heading={heading}");
        return true;
    }

    private static bool MoveForwardOneCell()
    {
        AlignHeading(targetHeading);

        double leftStart = wb_position_sensor_get_value(devices.LeftSensor);
        double rightStart = wb_position_sensor_get_value(devices.RightSensor);
        double dt = TimeStep / 1000.0;
        PidController yawPid = new PidController(ForwardYawKp, 0.0, ForwardYawKd, MaxForwardCorrection, 0.0);
        PidController centerWallPid = new PidController(WallCenterKp, 0.0, WallCenterKd, MaxLidarCorrection, 0.0);
        PidController singleWallPid = new PidController(OneWallKp, 0.0, OneWallKd, MaxLidarCorrection, 0.0);
        ResetLidarFilters();

        double lastFront = 0.0;
        double lastLeft = 0.0;
        double lastRight = 0.0;
        double maxCorrection = 0.0;

        while (wb_robot_step(TimeStep) != -1)
        {
            double travelled = ForwardDistance(leftStart, rightStart);
            double remaining = CellDistance - travelled;
            double baseSpeed = ForwardProfileSpeed(travelled, remaining);
            lastFront = Filter(ref filteredFront, ReadDistance(devices.FrontLidar));
            lastLeft = Filter(ref filteredLeft, ReadDistance(devices.LeftLidar));
            lastRight = Filter(ref filteredRight, ReadDistance(devices.RightLidar));

            if (lastFront > 0.005 && lastFront < FrontBlockedDistance && travelled < CellDistance * 0.70)
            {
                Stop();
                ReverseDistance(Math.Max(0.0, travelled - 0.010));
                Console.WriteLine($"Blocked while moving {directionText(heading)}: front={lastFront:F3} m, travelled={travelled:F3} m.");
                return false;
            }

            double yawError = ShortestAngle(ReadYaw(devices.Imu), targetHeading);
            double yawCorrection = yawPid.Update(yawError, dt);
            double wallCorrection = travelled > WallCorrectionStart && travelled < WallCorrectionEnd
                ? LidarCorrection(lastLeft, lastRight, centerWallPid, singleWallPid, dt)
                : 0.0;
            double correction = Clamp(yawCorrection + wallCorrection, MaxForwardCorrection);
            maxCorrection = Math.Max(maxCorrection, Math.Abs(correction));

            SetWheelSpeeds(baseSpeed + correction, baseSpeed - correction);

            if (travelled >= CellDistance)
            {
                Stop();
                AlignHeading(targetHeading);
                Settle();
                Console.WriteLine($"Move f: {travelled:F3} m, headingError={ShortestAngle(ReadYaw(devices.Imu), targetHeading):F3} rad, front={lastFront:F3}, left={lastLeft:F3}, right={lastRight:F3}, corrMax={maxCorrection:F2}");
                return true;
            }
        }

        return false;
    }

    private static void ReverseDistance(double distance)
    {
        if (distance <= 0.0)
            return;

        double leftStart = wb_position_sensor_get_value(devices.LeftSensor);
        double rightStart = wb_position_sensor_get_value(devices.RightSensor);
        while (wb_robot_step(TimeStep) != -1)
        {
            double backed = -ForwardDistance(leftStart, rightStart);
            SetWheelSpeeds(-0.65, -0.65);
            if (backed >= distance)
                break;
        }
        Stop();
        Settle();
        Console.WriteLine($"Reverse: {distance:F3} m");
    }

    private static void TurnToDirection(Direction targetDirection)
    {
        int delta = ((int)targetDirection - (int)heading + 4) % 4;
        if (delta == 0)
            return;

        if (delta == 1)
            TurnRight90();
        else if (delta == 3)
            TurnLeft90();
        else
        {
            TurnRight90();
            TurnRight90();
        }
    }

    private static void TurnLeft90()
    {
        targetHeading = NormalizeAngle(targetHeading + TurnAngle);
        TurnToTargetHeading("l");
        heading = LeftOf(heading);
    }

    private static void TurnRight90()
    {
        targetHeading = NormalizeAngle(targetHeading - TurnAngle);
        TurnToTargetHeading("r");
        heading = RightOf(heading);
    }

    private static void TurnToTargetHeading(string label)
    {
        double leftStart = wb_position_sensor_get_value(devices.LeftSensor);
        double rightStart = wb_position_sensor_get_value(devices.RightSensor);
        double startYaw = ReadYaw(devices.Imu);
        double dt = TimeStep / 1000.0;
        PidController turnPid = new PidController(TurnKp, 0.0, TurnKd, TurnSpeed, 0.0);
        int stableCount = 0;

        while (wb_robot_step(TimeStep) != -1)
        {
            double turnError = ShortestAngle(ReadYaw(devices.Imu), targetHeading);
            if (Math.Abs(turnError) <= TurnTolerance)
            {
                Stop();
                stableCount++;
                if (stableCount >= TurnStableSteps)
                    break;
                continue;
            }

            stableCount = 0;
            double turnCommand = -turnPid.Update(turnError, dt);
            if (Math.Abs(turnCommand) < MinTurnSpeed)
                turnCommand = Math.Sign(turnCommand) * MinTurnSpeed;
            turnCommand = Clamp(turnCommand, TurnSpeed);
            SetWheelSpeeds(-turnCommand, turnCommand);
        }

        Stop();
        Settle();
        Console.WriteLine($"Turn {label}: headingError={ShortestAngle(ReadYaw(devices.Imu), targetHeading):F3} rad, imuDelta={ShortestAngle(ReadYaw(devices.Imu), startYaw):F3} rad, encoder={HeadingChange(leftStart, rightStart):F3} rad");
    }

    private static void AlignHeading(double target)
    {
        int stableCount = 0;
        for (int i = 0; i < PreForwardAlignMaxSteps; i++)
        {
            double error = ShortestAngle(ReadYaw(devices.Imu), target);
            if (Math.Abs(error) <= PreForwardAlignTolerance)
            {
                Stop();
                stableCount++;
                if (stableCount >= PreForwardAlignStableSteps)
                    break;
                wb_robot_step(TimeStep);
                continue;
            }

            stableCount = 0;
            double command = Clamp(-2.0 * error, PreForwardAlignMaxSpeed);
            if (Math.Abs(command) < PreForwardAlignMinSpeed)
                command = Math.Sign(command) * PreForwardAlignMinSpeed;
            SetWheelSpeeds(-command, command);
            wb_robot_step(TimeStep);
        }

        Stop();
    }

    private static SensorReadings ReadStableDistances()
    {
        const int samples = 7;
        double[] front = new double[samples];
        double[] left = new double[samples];
        double[] right = new double[samples];

        for (int i = 0; i < samples; i++)
        {
            wb_robot_step(TimeStep);
            front[i] = ReadDistance(devices.FrontLidar);
            left[i] = ReadDistance(devices.LeftLidar);
            right[i] = ReadDistance(devices.RightLidar);
        }

        Array.Sort(front);
        Array.Sort(left);
        Array.Sort(right);
        return new SensorReadings(front[samples / 2], left[samples / 2], right[samples / 2]);
    }

    private static bool CanTraverse(int row, int col, Direction direction)
    {
        if (!InBounds(row, col))
            return false;
        int nr = row + RowStep[(int)direction];
        int nc = col + ColStep[(int)direction];
        return InBounds(nr, nc) && maze[row, col].Walls[(int)direction] == WallState.Open;
    }

    private static void SetEdge(int row, int col, Direction direction, WallState state, bool force)
    {
        if (!InBounds(row, col))
            return;

        int nr = row + RowStep[(int)direction];
        int nc = col + ColStep[(int)direction];
        if (!InBounds(nr, nc))
            state = WallState.Wall;

        SetLocalEdge(row, col, direction, state, force);
        if (InBounds(nr, nc))
            SetLocalEdge(nr, nc, Opposite(direction), state, force);
    }

    private static void SetLocalEdge(int row, int col, Direction direction, WallState state, bool force)
    {
        WallState old = maze[row, col].Walls[(int)direction];
        if (!force && old == WallState.Wall && state == WallState.Open)
            return;
        maze[row, col].Walls[(int)direction] = state;
    }

    private static List<Direction> ReconstructPath(int targetRow, int targetCol, int[,] parentRow, int[,] parentCol, Direction[,] parentDir)
    {
        List<Direction> reversed = new List<Direction>();
        int row = targetRow;
        int col = targetCol;

        while (!(row == currentRow && col == currentCol))
        {
            Direction direction = parentDir[row, col];
            reversed.Add(direction);
            int pr = parentRow[row, col];
            int pc = parentCol[row, col];
            if (pr < 0 || pc < 0)
                break;
            row = pr;
            col = pc;
        }

        reversed.Reverse();
        return reversed;
    }

    private static int[,] NewFilledArray(int rowCount, int colCount, int value)
    {
        int[,] result = new int[rowCount, colCount];
        for (int r = 0; r < rowCount; r++)
        {
            for (int c = 0; c < colCount; c++)
                result[r, c] = value;
        }
        return result;
    }

    private static void PrintMap()
    {
        Console.WriteLine($"--- mapping: visited {visitedCount}/{rows * cols} = {CompletionPercent():F1}% ---");
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                Console.Write("+");
                Console.Write(HorizontalWallText(maze[r, c].Walls[(int)Direction.North]));
            }
            Console.WriteLine("+");

            for (int c = 0; c < cols; c++)
            {
                Console.Write(VerticalWallText(maze[r, c].Walls[(int)Direction.West]));
                Console.Write(" ");
                Console.Write(CellMarker(r, c));
                Console.Write(" ");
            }
            Console.WriteLine(VerticalWallText(maze[r, cols - 1].Walls[(int)Direction.East]));
        }

        for (int c = 0; c < cols; c++)
        {
            Console.Write("+");
            Console.Write(HorizontalWallText(maze[rows - 1, c].Walls[(int)Direction.South]));
        }
        Console.WriteLine("+");
    }

    private static string HorizontalWallText(WallState state)
    {
        if (state == WallState.Wall)
            return "---";
        if (state == WallState.Open)
            return "   ";
        return " ? ";
    }

    private static string VerticalWallText(WallState state)
    {
        if (state == WallState.Wall)
            return "|";
        if (state == WallState.Open)
            return " ";
        return "?";
    }

    private static char CellMarker(int row, int col)
    {
        if (row == currentRow && col == currentCol)
        {
            if (heading == Direction.North)
                return '^';
            if (heading == Direction.East)
                return '>';
            if (heading == Direction.South)
                return 'v';
            return '<';
        }
        if (row == startRow && col == startCol)
            return 'S';
        if (row == goalRow && col == goalCol)
            return 'G';
        return maze[row, col].Visited ? '.' : '?';
    }

    private static double CompletionPercent()
    {
        return rows * cols == 0 ? 0.0 : 100.0 * visitedCount / (rows * cols);
    }

    private static double ForwardDistance(double leftStart, double rightStart)
    {
        return (LeftDistance(leftStart) + RightDistance(rightStart)) * 0.5;
    }

    private static double LeftDistance(double leftStart)
    {
        return (wb_position_sensor_get_value(devices.LeftSensor) - leftStart) * WheelRadius;
    }

    private static double RightDistance(double rightStart)
    {
        return (wb_position_sensor_get_value(devices.RightSensor) - rightStart) * WheelRadius;
    }

    private static double HeadingChange(double leftStart, double rightStart)
    {
        return (RightDistance(rightStart) - LeftDistance(leftStart)) / AxleLength;
    }

    private static double ForwardProfileSpeed(double travelled, double remaining)
    {
        const double startSpeed = 0.65;
        const double endSpeed = 0.35;
        double startRamp = SmoothStep(Clamp01(travelled / 0.050));
        double endRamp = SmoothStep(Clamp01(remaining / 0.075));
        double startLimited = startSpeed + startRamp * (ForwardSpeed - startSpeed);
        double endLimited = endSpeed + endRamp * (ForwardSpeed - endSpeed);
        return Math.Min(ForwardSpeed, Math.Min(startLimited, endLimited));
    }

    private static double LidarCorrection(double left, double right, PidController centerPid, PidController singleWallPid, double dt)
    {
        bool hasLeft = HasSideWall(left);
        bool hasRight = HasSideWall(right);

        if (hasLeft && hasRight)
        {
            singleWallPid.Reset();
            return centerPid.Update(right - left, dt);
        }

        centerPid.Reset();
        if (hasLeft && left < SingleWallUsableRange)
            return singleWallPid.Update(SideWallTarget - left, dt);
        if (hasRight && right < SingleWallUsableRange)
            return singleWallPid.Update(right - SideWallTarget, dt);

        singleWallPid.Reset();
        return 0.0;
    }

    private static bool HasSideWall(double distance)
    {
        return distance > 0.005 && distance < SideWallUsableRange;
    }

    private static double ReadYaw(int imu)
    {
        IntPtr ptr = wb_inertial_unit_get_roll_pitch_yaw(imu);
        double[] rpy = new double[3];
        Marshal.Copy(ptr, rpy, 0, 3);
        return rpy[2];
    }

    private static double ReadDistance(int sensor)
    {
        return wb_distance_sensor_get_value(sensor);
    }

    private static double Filter(ref double filtered, double sample)
    {
        if (double.IsNaN(filtered))
            filtered = sample;
        else
            filtered = 0.70 * filtered + 0.30 * sample;
        return filtered;
    }

    private static void ResetLidarFilters()
    {
        filteredFront = double.NaN;
        filteredLeft = double.NaN;
        filteredRight = double.NaN;
    }

    private static void Settle()
    {
        Stop();
        for (int i = 0; i < SettleSteps; i++)
            wb_robot_step(TimeStep);
    }

    private static Direction LeftOf(Direction direction)
    {
        return (Direction)(((int)direction + 3) % 4);
    }

    private static Direction RightOf(Direction direction)
    {
        return (Direction)(((int)direction + 1) % 4);
    }

    private static Direction Opposite(Direction direction)
    {
        return (Direction)(((int)direction + 2) % 4);
    }

    private static Direction DirectionCountSentinel()
    {
        return (Direction)4;
    }

    private static Direction ParseDirection(string text)
    {
        text = (text ?? string.Empty).Trim().ToUpperInvariant();
        if (text == "N" || text == "NORTH")
            return Direction.North;
        if (text == "S" || text == "SOUTH")
            return Direction.South;
        if (text == "W" || text == "WEST")
            return Direction.West;
        return Direction.East;
    }

    private static bool InBounds(int row, int col)
    {
        return row >= 0 && row < rows && col >= 0 && col < cols;
    }

    private static int ClampInt(int value, int min, int max)
    {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    private static double Clamp01(double value)
    {
        if (value < 0.0)
            return 0.0;
        if (value > 1.0)
            return 1.0;
        return value;
    }

    private static double SmoothStep(double value)
    {
        return value * value * (3.0 - 2.0 * value);
    }

    private static double Clamp(double value, double limit)
    {
        if (value > limit)
            return limit;
        if (value < -limit)
            return -limit;
        return value;
    }

    private static double ShortestAngle(double current, double reference)
    {
        double error = current - reference;
        while (error > Math.PI)
            error -= 2.0 * Math.PI;
        while (error < -Math.PI)
            error += 2.0 * Math.PI;
        return error;
    }

    private static double NormalizeAngle(double angle)
    {
        while (angle > Math.PI)
            angle -= 2.0 * Math.PI;
        while (angle < -Math.PI)
            angle += 2.0 * Math.PI;
        return angle;
    }

    private static string PtrToString(IntPtr ptr)
    {
        return ptr == IntPtr.Zero ? string.Empty : Marshal.PtrToStringAnsi(ptr) ?? string.Empty;
    }

    private static string directionText(Direction direction)
    {
        return direction.ToString();
    }

    private static void SetWheelSpeeds(double leftSpeed, double rightSpeed)
    {
        wb_motor_set_velocity(devices.LeftMotor, leftSpeed);
        wb_motor_set_velocity(devices.RightMotor, rightSpeed);
    }

    private static void Stop()
    {
        if (devices != null)
            SetWheelSpeeds(0.0, 0.0);
    }

    private sealed class CellInfo
    {
        public bool Visited;
        public readonly WallState[] Walls = { WallState.Unknown, WallState.Unknown, WallState.Unknown, WallState.Unknown };
    }

    private sealed class RobotDevices
    {
        public int LeftMotor;
        public int RightMotor;
        public int LeftSensor;
        public int RightSensor;
        public int Imu;
        public int FrontLidar;
        public int LeftLidar;
        public int RightLidar;
    }

    private sealed class RunnerConfig
    {
        public int Rows;
        public int Cols;
        public int StartRow;
        public int StartCol;
        public int GoalRow;
        public int GoalCol;
        public Direction Heading;
    }

    private readonly struct CellPoint
    {
        public CellPoint(int row, int col)
        {
            Row = row;
            Col = col;
        }

        public readonly int Row;
        public readonly int Col;
    }

    private readonly struct SensorReadings
    {
        public SensorReadings(double front, double left, double right)
        {
            Front = front;
            Left = left;
            Right = right;
        }

        public readonly double Front;
        public readonly double Left;
        public readonly double Right;
    }

    private sealed class PidController
    {
        private readonly double kp;
        private readonly double ki;
        private readonly double kd;
        private readonly double outputLimit;
        private readonly double integralLimit;
        private double integral;
        private double previousError;
        private bool hasPrevious;

        public PidController(double kp, double ki, double kd, double outputLimit, double integralLimit)
        {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.outputLimit = Math.Abs(outputLimit);
            this.integralLimit = Math.Abs(integralLimit);
        }

        public double Update(double error, double dt)
        {
            if (dt <= 0.0)
                dt = TimeStep / 1000.0;

            integral += error * dt;
            if (integralLimit > 0.0)
                integral = Clamp(integral, integralLimit);
            else
                integral = 0.0;

            double derivative = hasPrevious ? (error - previousError) / dt : 0.0;
            previousError = error;
            hasPrevious = true;
            return Clamp(kp * error + ki * integral + kd * derivative, outputLimit);
        }

        public void Reset()
        {
            integral = 0.0;
            previousError = 0.0;
            hasPrevious = false;
        }
    }
}
