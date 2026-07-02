using System;
using System.Runtime.InteropServices;
using System.Text;

internal static class Program
{
    private const int TimeStep = 32;
    private const double WheelRadius = 0.016;
    private const double AxleLength = 0.064;
    private const double CellDistance = 0.180;
    private const double TurnAngle = Math.PI / 2.0;
    private const double ForwardSpeed = 2.0;
    private const double TurnSpeed = 0.75;
    private const double MinTurnSpeed = 0.08;
    private const double TurnTolerance = 0.0035;
    private const int TurnStableSteps = 5;
    private const double LidarMaxRange = 0.16;
    private const double SideWallUsableRange = 0.130;
    private const double SingleWallUsableRange = 0.090;
    private const double SideWallTarget = 0.063;
    private const double MaxForwardCorrection = 0.22;
    private const double MaxLidarCorrection = 0.045;
    private const double FrontStopDistance = 0.045;
    private const double WallCorrectionStart = 0.015;
    private const double WallCorrectionEnd = CellDistance - 0.015;
    private const int SettleSteps = 10;
    private const string DefaultPath = "frfflflffrffrflfrflfrfflf";
    private const double ForwardYawKp = 2.4;
    private const double ForwardYawKd = 0.06;
    private const double TurnKp = 1.75;
    private const double TurnKd = 0.10;
    private const double WallCenterKp = 0.85;
    private const double WallCenterKd = 0.02;
    private const double OneWallKp = 0.55;
    private const double OneWallKd = 0.02;
    private const double PreForwardAlignTolerance = 0.006;
    private const double PreForwardAlignMaxSpeed = 0.22;
    private const double PreForwardAlignMinSpeed = 0.065;
    private const int PreForwardAlignStableSteps = 4;
    private const int PreForwardAlignMaxSteps = 50;
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

    private enum State
    {
        LoadCommand,
        Forward,
        TurnLeft,
        TurnRight,
        TurnToHeading,
        Settle,
        Done
    }

    private static int Main()
    {
        try
        {
            wb_robot_init();

            int leftMotor = wb_robot_get_device("left wheel motor");
            int rightMotor = wb_robot_get_device("right wheel motor");
            int leftSensor = wb_robot_get_device("left wheel sensor");
            int rightSensor = wb_robot_get_device("right wheel sensor");
            int imu = wb_robot_get_device("imu");
            int frontLidar = wb_robot_get_device("front lidar");
            int leftLidar = wb_robot_get_device("left lidar");
            int rightLidar = wb_robot_get_device("right lidar");

            wb_motor_set_position(leftMotor, double.PositiveInfinity);
            wb_motor_set_position(rightMotor, double.PositiveInfinity);
            SetWheelSpeeds(leftMotor, rightMotor, 0.0, 0.0);
            wb_position_sensor_enable(leftSensor, TimeStep);
            wb_position_sensor_enable(rightSensor, TimeStep);
            wb_inertial_unit_enable(imu, TimeStep);
            wb_distance_sensor_enable(frontLidar, TimeStep);
            wb_distance_sensor_enable(leftLidar, TimeStep);
            wb_distance_sensor_enable(rightLidar, TimeStep);

            wb_robot_step(TimeStep);

            string rawPath = PtrToString(wb_robot_get_custom_data());
            if (string.IsNullOrWhiteSpace(rawPath))
                rawPath = DefaultPath;

            string path = ExpandPath(rawPath);
            Console.WriteLine("Micromouse maze drive input: " + rawPath);
            Console.WriteLine("Micromouse maze drive expanded: " + path);

            int commandIndex = 0;
            int settleCount = 0;
            double leftStart = wb_position_sensor_get_value(leftSensor);
            double rightStart = wb_position_sensor_get_value(rightSensor);
            double targetHeading = ReadYaw(imu);
            double turnStart = targetHeading;
            double dt = TimeStep / 1000.0;
            PidController yawPid = new PidController(ForwardYawKp, 0.0, ForwardYawKd, MaxForwardCorrection, 0.0);
            PidController turnPid = new PidController(TurnKp, 0.0, TurnKd, TurnSpeed, 0.0);
            PidController centerWallPid = new PidController(WallCenterKp, 0.0, WallCenterKd, MaxLidarCorrection, 0.0);
            PidController singleWallPid = new PidController(OneWallKp, 0.0, OneWallKd, MaxLidarCorrection, 0.0);
            double maxYawPid = 0.0;
            double maxWallPid = 0.0;
            double lastActiveWallPid = 0.0;
            int turnStableCount = 0;
            State state = State.LoadCommand;

            while (wb_robot_step(TimeStep) != -1)
            {
                switch (state)
                {
                    case State.LoadCommand:
                        if (commandIndex >= path.Length)
                        {
                            Stop(leftMotor, rightMotor);
                            Console.WriteLine("Micromouse maze drive finished.");
                            state = State.Done;
                            break;
                        }

                        leftStart = wb_position_sensor_get_value(leftSensor);
                        rightStart = wb_position_sensor_get_value(rightSensor);
                        ResetLidarFilters();
                        yawPid.Reset();
                        turnPid.Reset();
                        centerWallPid.Reset();
                        singleWallPid.Reset();
                        maxYawPid = 0.0;
                        maxWallPid = 0.0;
                        lastActiveWallPid = 0.0;
                        turnStableCount = 0;

                        if (path[commandIndex] == 'f')
                        {
                            AlignHeading(leftMotor, rightMotor, imu, targetHeading);
                            leftStart = wb_position_sensor_get_value(leftSensor);
                            rightStart = wb_position_sensor_get_value(rightSensor);
                            ResetLidarFilters();
                            state = State.Forward;
                        }
                        else if (path[commandIndex] == 'l')
                        {
                            turnStart = ReadYaw(imu);
                            targetHeading = NormalizeAngle(targetHeading + TurnAngle);
                            state = State.TurnToHeading;
                        }
                        else if (path[commandIndex] == 'r')
                        {
                            turnStart = ReadYaw(imu);
                            targetHeading = NormalizeAngle(targetHeading - TurnAngle);
                            state = State.TurnToHeading;
                        }
                        else
                            commandIndex++;
                        break;

                    case State.Forward:
                        double travelled = ForwardDistance(leftSensor, rightSensor, leftStart, rightStart);
                        double remaining = CellDistance - travelled;
                        double baseSpeed = ForwardProfileSpeed(travelled, remaining);
                        double frontDistance = Filter(ref filteredFront, ReadDistance(frontLidar));
                        double leftDistance = Filter(ref filteredLeft, ReadDistance(leftLidar));
                        double rightDistance = Filter(ref filteredRight, ReadDistance(rightLidar));
                        double currentYaw = ReadYaw(imu);
                        double yawError = ShortestAngle(currentYaw, targetHeading);
                        double yawCorrection = yawPid.Update(yawError, dt);
                        bool safeForWallCorrection = !(frontDistance > 0.005 && frontDistance < 0.080);
                        double wallCorrection = safeForWallCorrection && travelled > WallCorrectionStart && travelled < WallCorrectionEnd
                            ? LidarCorrection(leftDistance, rightDistance, centerWallPid, singleWallPid, dt)
                            : 0.0;
                        double correction = Clamp(yawCorrection + wallCorrection, MaxForwardCorrection);
                        maxYawPid = Math.Max(maxYawPid, Math.Abs(yawCorrection));
                        if (Math.Abs(wallCorrection) > 1e-6)
                        {
                            lastActiveWallPid = wallCorrection;
                            maxWallPid = Math.Max(maxWallPid, Math.Abs(wallCorrection));
                        }
                        SetWheelSpeeds(leftMotor, rightMotor, baseSpeed + correction, baseSpeed - correction);
                        if (frontDistance > 0.005 && frontDistance < FrontStopDistance)
                        {
                            Console.WriteLine($"Front lidar warning: {frontDistance:F3} m");
                        }
                        if (travelled >= CellDistance)
                        {
                            Stop(leftMotor, rightMotor);
                            AlignHeading(leftMotor, rightMotor, imu, targetHeading);
                            Console.WriteLine($"Finished f {commandIndex + 1}/{path.Length}: {travelled:F3} m, headingError={ShortestAngle(ReadYaw(imu), targetHeading):F3} rad, left={leftDistance:F3} m, right={rightDistance:F3} m, yawMax={maxYawPid:F2}, wallLast={lastActiveWallPid:F2}, wallMax={maxWallPid:F2}, corr={correction:F2}");
                            settleCount = 0;
                            state = State.Settle;
                        }
                        break;

                    case State.TurnLeft:
                    case State.TurnRight:
                    case State.TurnToHeading:
                        double turnError = ShortestAngle(ReadYaw(imu), targetHeading);
                        if (Math.Abs(turnError) <= TurnTolerance)
                        {
                            Stop(leftMotor, rightMotor);
                            turnStableCount++;
                            if (turnStableCount >= TurnStableSteps)
                            {
                                Console.WriteLine($"Finished {path[commandIndex]} {commandIndex + 1}/{path.Length}: headingError={ShortestAngle(ReadYaw(imu), targetHeading):F3} rad, imuDelta={ShortestAngle(ReadYaw(imu), turnStart):F3} rad, encoder={HeadingChange(leftSensor, rightSensor, leftStart, rightStart):F3} rad");
                                settleCount = 0;
                                state = State.Settle;
                            }
                        }
                        else
                        {
                            turnStableCount = 0;
                            double turnCommand = -turnPid.Update(turnError, dt);
                            if (Math.Abs(turnCommand) < MinTurnSpeed)
                                turnCommand = Math.Sign(turnCommand) * MinTurnSpeed;
                            turnCommand = Clamp(turnCommand, TurnSpeed);
                            SetWheelSpeeds(leftMotor, rightMotor, -turnCommand, turnCommand);
                        }
                        break;

                    case State.Settle:
                        Stop(leftMotor, rightMotor);
                        settleCount++;
                        if (settleCount >= SettleSteps)
                        {
                            commandIndex++;
                            state = State.LoadCommand;
                        }
                        break;

                    case State.Done:
                        Stop(leftMotor, rightMotor);
                        break;
                }
            }

            wb_robot_cleanup();
            return 0;
        }
        catch (Exception ex)
        {
            Console.WriteLine("micromouse_maze_drive failed: " + ex);
            return 1;
        }
    }

    private static double LeftDistance(int leftSensor, double leftStart)
    {
        return (wb_position_sensor_get_value(leftSensor) - leftStart) * WheelRadius;
    }

    private static double RightDistance(int rightSensor, double rightStart)
    {
        return (wb_position_sensor_get_value(rightSensor) - rightStart) * WheelRadius;
    }

    private static double ForwardDistance(int leftSensor, int rightSensor, double leftStart, double rightStart)
    {
        return (LeftDistance(leftSensor, leftStart) + RightDistance(rightSensor, rightStart)) * 0.5;
    }

    private static double ForwardProfileSpeed(double travelled, double remaining)
    {
        const double startSpeed = 0.80;
        const double endSpeed = 0.45;
        double startRamp = SmoothStep(Clamp01(travelled / 0.045));
        double endRamp = SmoothStep(Clamp01(remaining / 0.080));
        double startLimited = startSpeed + startRamp * (ForwardSpeed - startSpeed);
        double endLimited = endSpeed + endRamp * (ForwardSpeed - endSpeed);

        return Math.Min(ForwardSpeed, Math.Min(startLimited, endLimited));
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

    private static double HeadingChange(int leftSensor, int rightSensor, double leftStart, double rightStart)
    {
        return (RightDistance(rightSensor, rightStart) - LeftDistance(leftSensor, leftStart)) / AxleLength;
    }

    private static void AlignHeading(int leftMotor, int rightMotor, int imu, double targetHeading)
    {
        int stableCount = 0;
        for (int i = 0; i < PreForwardAlignMaxSteps; i++)
        {
            double error = ShortestAngle(ReadYaw(imu), targetHeading);
            if (Math.Abs(error) <= PreForwardAlignTolerance)
            {
                Stop(leftMotor, rightMotor);
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
            SetWheelSpeeds(leftMotor, rightMotor, -command, command);
            wb_robot_step(TimeStep);
        }

        Stop(leftMotor, rightMotor);
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

    private static bool HasWall(double distance)
    {
        return distance > 0.005 && distance < SideWallUsableRange;
    }

    private static double LidarCorrection(double left, double right, PidController centerPid, PidController singleWallPid, double dt)
    {
        bool hasLeft = HasWall(left);
        bool hasRight = HasWall(right);

        if (hasLeft && hasRight)
        {
            singleWallPid.Reset();
            return centerPid.Update(right - left, dt);
        }

        centerPid.Reset();
        if (hasLeft)
        {
            if (left < SingleWallUsableRange)
                return singleWallPid.Update(SideWallTarget - left, dt);
            singleWallPid.Reset();
            return 0.0;
        }
        if (hasRight)
        {
            if (right < SingleWallUsableRange)
                return singleWallPid.Update(right - SideWallTarget, dt);
            singleWallPid.Reset();
            return 0.0;
        }

        singleWallPid.Reset();
        return 0.0;
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

    private static void SetWheelSpeeds(int leftMotor, int rightMotor, double leftSpeed, double rightSpeed)
    {
        wb_motor_set_velocity(leftMotor, leftSpeed);
        wb_motor_set_velocity(rightMotor, rightSpeed);
    }

    private static void Stop(int leftMotor, int rightMotor)
    {
        SetWheelSpeeds(leftMotor, rightMotor, 0.0, 0.0);
    }

    private static string PtrToString(IntPtr ptr)
    {
        return ptr == IntPtr.Zero ? string.Empty : Marshal.PtrToStringAnsi(ptr) ?? string.Empty;
    }

    private static string ExpandPath(string input)
    {
        input = input.ToLowerInvariant().Replace("path=", " ");
        StringBuilder output = new StringBuilder();

        for (int i = 0; i < input.Length; i++)
        {
            char command = input[i];
            if (command != 'f' && command != 'l' && command != 'r')
                continue;

            int repeat = 0;
            while (i + 1 < input.Length && char.IsDigit(input[i + 1]))
            {
                repeat = repeat * 10 + input[i + 1] - '0';
                i++;
            }
            if (repeat == 0)
                repeat = 1;

            for (int r = 0; r < repeat; r++)
                output.Append(command);
        }

        return output.ToString();
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
