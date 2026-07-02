using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

internal static class Program
{
    private const int TimeStep = 32;
    private const int Rows = 5;
    private const int Cols = 9;
    private const int StartRow = 0;
    private const int StartCol = 0;
    private const Dir StartHeading = Dir.East;
    private const int GoalRow = 4;
    private const int GoalCol = 7;

    private const double WheelRadius = 0.016;
    private const double AxleLength = 0.064;
    private const double CellDistance = 0.180;
    private const double TurnAngle = Math.PI / 2.0;
    private const double ForwardSpeed = 3.0;
    private const double TurnSpeed = 1.2;
    private const double MinTurnSpeed = 0.28;
    private const double TurnHeadingGain = 2.2;
    private const double TurnTolerance = 0.012;
    private const double LidarWallDistance = 0.160;
    private const double SideWallTarget = 0.063;
    private const double YawHoldGain = 3.8;
    private const double CenteringGain = 5.0;
    private const double OneWallGain = 4.0;
    private const double MaxForwardCorrection = 0.45;
    private const double MaxLidarCorrection = 0.25;
    private const double FrontStopDistance = 0.045;
    private const double BlockedFrontDistance = 0.105;
    private const double BlockedCellMargin = 0.030;
    private const double ProbeDistance = 0.025;
    private const double ProbeWallDistance = 0.115;
    private const double ProbeSpeed = 0.55;
    private const double WallCorrectionStart = 0.035;
    private const double WallCorrectionEnd = CellDistance - 0.035;
    private const int SettleSteps = 10;

    private static readonly int[] RowStep = { -1, 0, 1, 0 };
    private static readonly int[] ColStep = { 0, 1, 0, -1 };
    private static readonly Dir[] ExplorationOrder = { Dir.North, Dir.East, Dir.South, Dir.West };

    private static readonly Cell[,] Maze = new Cell[Rows, Cols];

    private static int leftMotor;
    private static int rightMotor;
    private static int leftSensor;
    private static int rightSensor;
    private static int imu;
    private static int frontLidar;
    private static int leftLidar;
    private static int rightLidar;

    private static double targetHeading;
    private static double filteredFront = double.NaN;
    private static double filteredLeft = double.NaN;
    private static double filteredRight = double.NaN;

    private enum Dir
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

    private sealed class Cell
    {
        public bool Visited;
        public int Distance;
        public readonly WallState[] Walls = { WallState.Unknown, WallState.Unknown, WallState.Unknown, WallState.Unknown };
    }

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_robot_init();

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern int wb_robot_step(int duration);

    [DllImport("Controller", CallingConvention = CallingConvention.Cdecl)]
    private static extern void wb_robot_cleanup();

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

    private static int Main()
    {
        try
        {
            wb_robot_init();
            InitDevices();
            InitMaze();
            wb_robot_step(TimeStep);

            targetHeading = ReadYaw();
            RunAutonomousMapping();

            while (wb_robot_step(TimeStep) != -1)
                Stop();

            wb_robot_cleanup();
            return 0;
        }
        catch (Exception ex)
        {
            Console.WriteLine("autonomous_mapper failed: " + ex);
            Stop();
            return 1;
        }
    }

    private static void InitDevices()
    {
        leftMotor = wb_robot_get_device("left wheel motor");
        rightMotor = wb_robot_get_device("right wheel motor");
        leftSensor = wb_robot_get_device("left wheel sensor");
        rightSensor = wb_robot_get_device("right wheel sensor");
        imu = wb_robot_get_device("imu");
        frontLidar = wb_robot_get_device("front lidar");
        leftLidar = wb_robot_get_device("left lidar");
        rightLidar = wb_robot_get_device("right lidar");

        wb_motor_set_position(leftMotor, double.PositiveInfinity);
        wb_motor_set_position(rightMotor, double.PositiveInfinity);
        Stop();
        wb_position_sensor_enable(leftSensor, TimeStep);
        wb_position_sensor_enable(rightSensor, TimeStep);
        wb_inertial_unit_enable(imu, TimeStep);
        wb_distance_sensor_enable(frontLidar, TimeStep);
        wb_distance_sensor_enable(leftLidar, TimeStep);
        wb_distance_sensor_enable(rightLidar, TimeStep);
    }

    private static void InitMaze()
    {
        for (int row = 0; row < Rows; row++)
        {
            for (int col = 0; col < Cols; col++)
            {
                Maze[row, col] = new Cell();
                Maze[row, col].Distance = Rows * Cols;
                for (int dir = 0; dir < 4; dir++)
                {
                    int nr = row + RowStep[dir];
                    int nc = col + ColStep[dir];
                    if (!InBounds(nr, nc))
                        Maze[row, col].Walls[dir] = WallState.Wall;
                }
            }
        }
    }

    private static void RunAutonomousMapping()
    {
        int row = StartRow;
        int col = StartCol;
        Dir heading = StartHeading;

        Console.WriteLine("Autonomous mapper started.");
        Console.WriteLine($"Start=({StartRow},{StartCol},{StartHeading}), Goal=({GoalRow},{GoalCol}), Maze={Rows}x{Cols}");

        int maxExploreSteps = Rows * Cols * 12;
        for (int step = 0; step < maxExploreSteps && CountVisited() < Rows * Cols; step++)
        {
            ScanCell(row, col, heading);
            if (CountVisited() >= Rows * Cols)
                break;

            FloodDistancesToUnvisited();
            PrintMap(row, col, heading, "full mapping");

            Dir? next = ChooseFloodDirection(row, col, heading);
            if (!next.HasValue)
            {
                Console.WriteLine("No flood-fill candidate to any unvisited cell is available; mapping cannot continue.");
                break;
            }

            if (!MoveToNeighbor(next.Value, ref row, ref col, ref heading))
                step--;
        }

        ScanCell(row, col, heading);
        PrintMap(row, col, heading, "mapping complete check");

        if (CountVisited() < Rows * Cols)
        {
            Console.WriteLine($"Mapping stopped at {CountVisited()}/{Rows * Cols}; shortest run will not start because completion is not 100%.");
            Stop();
            return;
        }

        Console.WriteLine("Mapping reached 100%. Returning to start over confirmed open passages.");
        List<Dir> returnPath = BfsDirections(row, col, StartRow, StartCol);
        if (returnPath.Count == 0 && !(row == StartRow && col == StartCol))
        {
            Console.WriteLine("No confirmed return path to start was found.");
            Stop();
            return;
        }

        foreach (Dir dir in returnPath)
        {
            if (!MoveToNeighbor(dir, ref row, ref col, ref heading))
                break;
            PrintMap(row, col, heading, "return to start");
        }

        Console.WriteLine("Calculating shortest path from start to goal using confirmed map.");

        List<Dir> shortestPath = BfsDirections(StartRow, StartCol, GoalRow, GoalCol);
        if (shortestPath.Count == 0)
        {
            Console.WriteLine("No route to goal was found in the discovered map.");
            return;
        }

        Console.WriteLine("Shortest path directions: " + DirectionsToText(shortestPath));

        foreach (Dir dir in shortestPath)
        {
            if (!MoveToNeighbor(dir, ref row, ref col, ref heading))
                break;
            PrintMap(row, col, heading, "shortest run");
        }

        Console.WriteLine($"Shortest path run finished at ({row},{col}).");
        Stop();
    }

    private static void ScanCell(int row, int col, Dir heading)
    {
        Maze[row, col].Visited = true;
        ResetLidarFilters();
        for (int i = 0; i < 5; i++)
            wb_robot_step(TimeStep);

        double front = MedianDistance(frontLidar);
        double left = MedianDistance(leftLidar);
        double right = MedianDistance(rightLidar);

        MarkScannedWall(row, col, heading, front);
        MarkScannedWall(row, col, TurnLeft(heading), left);
        MarkScannedWall(row, col, TurnRight(heading), right);

        Console.WriteLine($"Scan ({row},{col}) heading={heading}: front={front:F3}, left={left:F3}, right={right:F3}");
    }

    private static void FloodDistances(int targetRow, int targetCol)
    {
        for (int row = 0; row < Rows; row++)
            for (int col = 0; col < Cols; col++)
                Maze[row, col].Distance = Rows * Cols;

        Queue<(int Row, int Col)> queue = new Queue<(int Row, int Col)>();
        Maze[targetRow, targetCol].Distance = 0;
        queue.Enqueue((targetRow, targetCol));

        while (queue.Count > 0)
        {
            (int Row, int Col) current = queue.Dequeue();
            int nextDistance = Maze[current.Row, current.Col].Distance + 1;

            for (int dir = 0; dir < 4; dir++)
            {
                int nr = current.Row + RowStep[dir];
                int nc = current.Col + ColStep[dir];
                if (!InBounds(nr, nc))
                    continue;

                Dir reverseDir = Opposite((Dir)dir);
                if (Maze[current.Row, current.Col].Walls[dir] == WallState.Wall || Maze[nr, nc].Walls[(int)reverseDir] == WallState.Wall)
                    continue;

                if (nextDistance < Maze[nr, nc].Distance)
                {
                    Maze[nr, nc].Distance = nextDistance;
                    queue.Enqueue((nr, nc));
                }
            }
        }
    }

    private static void FloodDistancesToUnvisited()
    {
        for (int row = 0; row < Rows; row++)
            for (int col = 0; col < Cols; col++)
                Maze[row, col].Distance = Rows * Cols;

        Queue<(int Row, int Col)> queue = new Queue<(int Row, int Col)>();
        for (int row = 0; row < Rows; row++)
        {
            for (int col = 0; col < Cols; col++)
            {
                if (!Maze[row, col].Visited)
                {
                    Maze[row, col].Distance = 0;
                    queue.Enqueue((row, col));
                }
            }
        }

        while (queue.Count > 0)
        {
            (int Row, int Col) current = queue.Dequeue();
            int nextDistance = Maze[current.Row, current.Col].Distance + 1;

            for (int dir = 0; dir < 4; dir++)
            {
                int nr = current.Row + RowStep[dir];
                int nc = current.Col + ColStep[dir];
                if (!InBounds(nr, nc))
                    continue;

                Dir reverseDir = Opposite((Dir)dir);
                if (Maze[current.Row, current.Col].Walls[dir] == WallState.Wall || Maze[nr, nc].Walls[(int)reverseDir] == WallState.Wall)
                    continue;

                if (nextDistance < Maze[nr, nc].Distance)
                {
                    Maze[nr, nc].Distance = nextDistance;
                    queue.Enqueue((nr, nc));
                }
            }
        }
    }

    private static Dir? ChooseFloodDirection(int row, int col, Dir heading)
    {
        Dir[] priority = { heading, TurnRight(heading), TurnLeft(heading), Opposite(heading) };
        Dir? best = null;
        int bestDistance = Rows * Cols + 1;
        int bestKnownPenalty = 1000;

        foreach (Dir dir in priority)
        {
            int nr = row + RowStep[(int)dir];
            int nc = col + ColStep[(int)dir];
            if (!InBounds(nr, nc) || Maze[row, col].Walls[(int)dir] == WallState.Wall)
                continue;

            int knownPenalty = Maze[row, col].Walls[(int)dir] == WallState.Open ? 0 : 1;
            int distance = Maze[nr, nc].Distance;
            if (distance < bestDistance || (distance == bestDistance && knownPenalty < bestKnownPenalty))
            {
                best = dir;
                bestDistance = distance;
                bestKnownPenalty = knownPenalty;
            }
        }

        if (best.HasValue)
            Console.WriteLine($"Flood choice: {best.Value}, distance={bestDistance}, edge={Maze[row, col].Walls[(int)best.Value]}");
        return best;
    }

    private static bool MoveToNeighbor(Dir targetDir, ref int row, ref int col, ref Dir heading)
    {
        TurnToDirection(targetDir, ref heading);
        if (Maze[row, col].Walls[(int)targetDir] != WallState.Open && !ProbeForwardEdge())
        {
            SetWall(row, col, targetDir, true);
            Console.WriteLine($"Probe found wall at ({row},{col}) -> {targetDir}; marked as wall and replanning.");
            return false;
        }

        if (!MoveForwardOneCell())
        {
            SetWall(row, col, targetDir, true);
            Console.WriteLine($"Blocked edge detected at ({row},{col}) -> {targetDir}; marked as wall and replanning.");
            return false;
        }

        MarkPassage(row, col, targetDir);
        row += RowStep[(int)targetDir];
        col += ColStep[(int)targetDir];
        Console.WriteLine($"Pose update: ({row},{col}) heading={heading}");
        return true;
    }

    private static bool ProbeForwardEdge()
    {
        ResetLidarFilters();
        for (int i = 0; i < 3; i++)
            wb_robot_step(TimeStep);

        double initialFront = MedianDistance(frontLidar);
        if (initialFront > 0.005 && initialFront < ProbeWallDistance)
        {
            Console.WriteLine($"Probe immediate wall: front={initialFront:F3} m");
            return false;
        }

        double leftStart = wb_position_sensor_get_value(leftSensor);
        double rightStart = wb_position_sensor_get_value(rightSensor);

        while (wb_robot_step(TimeStep) != -1)
        {
            double travelled = ForwardDistance(leftStart, rightStart);
            double frontDistance = Filter(ref filteredFront, ReadDistance(frontLidar));
            double yawCorrection = YawHoldGain * ShortestAngle(ReadYaw(), targetHeading);
            double correction = Clamp(yawCorrection, MaxForwardCorrection);

            if (frontDistance > 0.005 && frontDistance < ProbeWallDistance)
            {
                Stop();
                Console.WriteLine($"Probe blocked: front={frontDistance:F3} m, travelled={travelled:F3} m");
                ReverseDistance(Math.Max(0.0, travelled - 0.003));
                Settle();
                return false;
            }

            SetWheelSpeeds(ProbeSpeed + correction, ProbeSpeed - correction);

            if (travelled >= ProbeDistance)
            {
                Stop();
                Console.WriteLine($"Probe open: {travelled:F3} m");
                ReverseDistance(travelled);
                Settle();
                return true;
            }
        }

        return false;
    }

    private static void TurnToDirection(Dir targetDir, ref Dir heading)
    {
        int delta = ((int)targetDir - (int)heading + 4) % 4;
        if (delta == 1)
        {
            TurnRight90();
            heading = TurnRight(heading);
        }
        else if (delta == 2)
        {
            TurnRight90();
            heading = TurnRight(heading);
            TurnRight90();
            heading = TurnRight(heading);
        }
        else if (delta == 3)
        {
            TurnLeft90();
            heading = TurnLeft(heading);
        }
    }

    private static bool MoveForwardOneCell()
    {
        double leftStart = wb_position_sensor_get_value(leftSensor);
        double rightStart = wb_position_sensor_get_value(rightSensor);
        ResetLidarFilters();

        while (wb_robot_step(TimeStep) != -1)
        {
            double travelled = ForwardDistance(leftStart, rightStart);
            double remaining = CellDistance - travelled;
            double baseSpeed = ForwardProfileSpeed(travelled, remaining);
            double frontDistance = Filter(ref filteredFront, ReadDistance(frontLidar));
            double leftDistance = Filter(ref filteredLeft, ReadDistance(leftLidar));
            double rightDistance = Filter(ref filteredRight, ReadDistance(rightLidar));
            double yawCorrection = YawHoldGain * ShortestAngle(ReadYaw(), targetHeading);
            double wallCorrection = travelled > WallCorrectionStart && travelled < WallCorrectionEnd
                ? LidarCorrection(leftDistance, rightDistance)
                : 0.0;
            double correction = Clamp(yawCorrection + wallCorrection, MaxForwardCorrection);

            if (frontDistance > 0.005 && frontDistance < BlockedFrontDistance && travelled < CellDistance - BlockedCellMargin)
            {
                Stop();
                Console.WriteLine($"Blocked during f: front={frontDistance:F3} m, travelled={travelled:F3} m. Reversing to cell centre.");
                ReverseDistance(Math.Max(0.0, travelled - 0.004));
                Settle();
                return false;
            }

            SetWheelSpeeds(baseSpeed + correction, baseSpeed - correction);

            if (frontDistance > 0.005 && frontDistance < FrontStopDistance && travelled > 0.060)
                Console.WriteLine($"Front warning while moving: {frontDistance:F3} m");

            if (travelled >= CellDistance)
            {
                Stop();
                Console.WriteLine($"Move f: {travelled:F3} m, headingError={ShortestAngle(ReadYaw(), targetHeading):F3} rad, left={leftDistance:F3}, right={rightDistance:F3}, corr={correction:F2}");
                Settle();
                return true;
            }
        }

        return false;
    }

    private static void ReverseDistance(double distance)
    {
        if (distance <= 0.001)
            return;

        double leftStart = wb_position_sensor_get_value(leftSensor);
        double rightStart = wb_position_sensor_get_value(rightSensor);
        double target = Math.Min(distance, CellDistance * 0.85);

        while (wb_robot_step(TimeStep) != -1)
        {
            double travelled = -ForwardDistance(leftStart, rightStart);
            double remaining = target - travelled;
            double speed = remaining < 0.030 ? 0.9 : 1.4;
            double yawCorrection = YawHoldGain * ShortestAngle(ReadYaw(), targetHeading);
            double correction = Clamp(yawCorrection, MaxForwardCorrection);

            SetWheelSpeeds(-speed + correction, -speed - correction);

            if (travelled >= target)
            {
                Stop();
                Console.WriteLine($"Reverse: {travelled:F3} m");
                return;
            }
        }
    }

    private static void TurnLeft90()
    {
        targetHeading = NormalizeAngle(targetHeading + TurnAngle);
        TurnToTarget("l");
    }

    private static void TurnRight90()
    {
        targetHeading = NormalizeAngle(targetHeading - TurnAngle);
        TurnToTarget("r");
    }

    private static void TurnToTarget(string label)
    {
        double leftStart = wb_position_sensor_get_value(leftSensor);
        double rightStart = wb_position_sensor_get_value(rightSensor);
        double turnStart = ReadYaw();

        while (wb_robot_step(TimeStep) != -1)
        {
            double error = ShortestAngle(ReadYaw(), targetHeading);
            if (Math.Abs(error) <= TurnTolerance)
            {
                Stop();
                Console.WriteLine($"Turn {label}: headingError={error:F3} rad, imuDelta={ShortestAngle(ReadYaw(), turnStart):F3} rad, encoder={HeadingChange(leftStart, rightStart):F3} rad");
                Settle();
                return;
            }

            double turnCommand = Clamp(-TurnHeadingGain * error, TurnSpeed);
            if (Math.Abs(turnCommand) < MinTurnSpeed)
                turnCommand = Math.Sign(turnCommand) * MinTurnSpeed;
            SetWheelSpeeds(-turnCommand, turnCommand);
        }
    }

    private static List<Dir> BfsDirections(int startRow, int startCol, int goalRow, int goalCol)
    {
        bool[,] seen = new bool[Rows, Cols];
        (int Row, int Col)[,] previous = new (int Row, int Col)[Rows, Cols];
        Queue<(int Row, int Col)> queue = new Queue<(int Row, int Col)>();
        queue.Enqueue((startRow, startCol));
        seen[startRow, startCol] = true;
        previous[startRow, startCol] = (-1, -1);

        while (queue.Count > 0)
        {
            (int Row, int Col) current = queue.Dequeue();
            if (current.Row == goalRow && current.Col == goalCol)
                break;

            for (int dir = 0; dir < 4; dir++)
            {
                int nr = current.Row + RowStep[dir];
                int nc = current.Col + ColStep[dir];
                if (!InBounds(nr, nc) || seen[nr, nc] || Maze[current.Row, current.Col].Walls[dir] != WallState.Open)
                    continue;
                seen[nr, nc] = true;
                previous[nr, nc] = current;
                queue.Enqueue((nr, nc));
            }
        }

        List<Dir> result = new List<Dir>();
        if (!seen[goalRow, goalCol])
            return result;

        int row = goalRow;
        int col = goalCol;
        while (!(row == startRow && col == startCol))
        {
            (int Row, int Col) prev = previous[row, col];
            result.Add(DirectionBetween(prev.Row, prev.Col, row, col));
            row = prev.Row;
            col = prev.Col;
        }

        result.Reverse();
        return result;
    }

    private static void PrintMap(int robotRow, int robotCol, Dir heading, string phase)
    {
        int visited = CountVisited();
        double completion = 100.0 * visited / (Rows * Cols);
        Console.WriteLine($"--- {phase}: visited {visited}/{Rows * Cols} = {completion:F1}% ---");

        for (int row = 0; row < Rows; row++)
        {
            for (int col = 0; col < Cols; col++)
                Console.Write("+" + (Maze[row, col].Walls[(int)Dir.North] == WallState.Wall ? "---" : "   "));
            Console.WriteLine("+");

            for (int col = 0; col < Cols; col++)
            {
                Console.Write(Maze[row, col].Walls[(int)Dir.West] == WallState.Wall ? "|" : " ");
                Console.Write(" " + CellChar(row, col, robotRow, robotCol, heading) + " ");
            }
            Console.WriteLine(Maze[row, Cols - 1].Walls[(int)Dir.East] == WallState.Wall ? "|" : " ");
        }

        for (int col = 0; col < Cols; col++)
            Console.Write("+" + (Maze[Rows - 1, col].Walls[(int)Dir.South] == WallState.Wall ? "---" : "   "));
        Console.WriteLine("+");
    }

    private static char CellChar(int row, int col, int robotRow, int robotCol, Dir heading)
    {
        if (row == robotRow && col == robotCol)
        {
            if (heading == Dir.North)
                return '^';
            if (heading == Dir.East)
                return '>';
            if (heading == Dir.South)
                return 'v';
            return '<';
        }
        if (row == StartRow && col == StartCol)
            return 'S';
        if (row == GoalRow && col == GoalCol)
            return 'G';
        return Maze[row, col].Visited ? '.' : '?';
    }

    private static int CountVisited()
    {
        int count = 0;
        for (int row = 0; row < Rows; row++)
            for (int col = 0; col < Cols; col++)
                if (Maze[row, col].Visited)
                    count++;
        return count;
    }

    private static string DirectionsToText(List<Dir> directions)
    {
        char[] chars = new char[directions.Count];
        for (int i = 0; i < directions.Count; i++)
            chars[i] = directions[i] == Dir.North ? 'N' : directions[i] == Dir.East ? 'E' : directions[i] == Dir.South ? 'S' : 'W';
        return new string(chars);
    }

    private static void SetWall(int row, int col, Dir dir, bool wall)
    {
        int nr = row + RowStep[(int)dir];
        int nc = col + ColStep[(int)dir];
        if (!InBounds(nr, nc))
        {
            Maze[row, col].Walls[(int)dir] = WallState.Wall;
            return;
        }

        WallState state = wall ? WallState.Wall : WallState.Open;
        if (!wall && Maze[row, col].Walls[(int)dir] == WallState.Wall)
            return;

        Maze[row, col].Walls[(int)dir] = state;
        Maze[nr, nc].Walls[(int)Opposite(dir)] = state;
    }

    private static void MarkScannedWall(int row, int col, Dir dir, double distance)
    {
        if (IsWall(distance))
            SetWall(row, col, dir, true);
    }

    private static void MarkPassage(int row, int col, Dir dir)
    {
        int nr = row + RowStep[(int)dir];
        int nc = col + ColStep[(int)dir];
        if (!InBounds(nr, nc))
            return;

        Maze[row, col].Walls[(int)dir] = WallState.Open;
        Maze[nr, nc].Walls[(int)Opposite(dir)] = WallState.Open;
    }

    private static Dir DirectionBetween(int row, int col, int nextRow, int nextCol)
    {
        for (int dir = 0; dir < 4; dir++)
        {
            if (row + RowStep[dir] == nextRow && col + ColStep[dir] == nextCol)
                return (Dir)dir;
        }

        throw new InvalidOperationException("Cells are not neighbours.");
    }

    private static bool InBounds(int row, int col)
    {
        return row >= 0 && row < Rows && col >= 0 && col < Cols;
    }

    private static Dir TurnLeft(Dir dir)
    {
        return (Dir)(((int)dir + 3) % 4);
    }

    private static Dir TurnRight(Dir dir)
    {
        return (Dir)(((int)dir + 1) % 4);
    }

    private static Dir Opposite(Dir dir)
    {
        return (Dir)(((int)dir + 2) % 4);
    }

    private static double MedianDistance(int sensor)
    {
        double[] samples = new double[5];
        for (int i = 0; i < samples.Length; i++)
        {
            wb_robot_step(TimeStep);
            samples[i] = ReadDistance(sensor);
        }
        Array.Sort(samples);
        return samples[samples.Length / 2];
    }

    private static bool IsWall(double distance)
    {
        return distance > 0.005 && distance < LidarWallDistance;
    }

    private static double ForwardDistance(double leftStart, double rightStart)
    {
        return (LeftDistance(leftStart) + RightDistance(rightStart)) * 0.5;
    }

    private static double LeftDistance(double leftStart)
    {
        return (wb_position_sensor_get_value(leftSensor) - leftStart) * WheelRadius;
    }

    private static double RightDistance(double rightStart)
    {
        return (wb_position_sensor_get_value(rightSensor) - rightStart) * WheelRadius;
    }

    private static double HeadingChange(double leftStart, double rightStart)
    {
        return (RightDistance(rightStart) - LeftDistance(leftStart)) / AxleLength;
    }

    private static double ForwardProfileSpeed(double travelled, double remaining)
    {
        double speed = ForwardSpeed;

        if (travelled < 0.025)
            speed = Math.Min(speed, 1.6 + travelled / 0.025 * (ForwardSpeed - 1.6));
        if (remaining < 0.060)
            speed = Math.Min(speed, 2.1);
        if (remaining < 0.030)
            speed = Math.Min(speed, 1.4);
        if (remaining < 0.012)
            speed = Math.Min(speed, 0.8);

        return speed;
    }

    private static double ReadYaw()
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

    private static double LidarCorrection(double left, double right)
    {
        bool hasLeft = IsWall(left);
        bool hasRight = IsWall(right);
        double correction = 0.0;

        if (hasLeft && hasRight)
            correction = CenteringGain * (right - left);
        else if (hasLeft)
            correction = OneWallGain * (SideWallTarget - left);
        else if (hasRight)
            correction = OneWallGain * (right - SideWallTarget);

        return Clamp(correction, MaxLidarCorrection);
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

    private static void Settle()
    {
        for (int i = 0; i < SettleSteps; i++)
        {
            Stop();
            wb_robot_step(TimeStep);
        }
    }

    private static void SetWheelSpeeds(double leftSpeed, double rightSpeed)
    {
        wb_motor_set_velocity(leftMotor, leftSpeed);
        wb_motor_set_velocity(rightMotor, rightSpeed);
    }

    private static void Stop()
    {
        SetWheelSpeeds(0.0, 0.0);
    }
}
