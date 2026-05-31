namespace Maphy.Physics
{
    public static class ConstraintRegistry
    {
        public static bool IsImplemented(ConstraintType type)
        {
            return GetDescriptor(type).implemented;
        }

        public static ConstraintDescriptor GetDescriptor(ConstraintType type)
        {
            switch (type)
            {
                case ConstraintType.Distance:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.LinearLimit, true);
                case ConstraintType.Point:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.LinearLimit, true);
                case ConstraintType.SpringDistance:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.LinearLimit | ConstraintCapabilities.Spring, true);
                case ConstraintType.Hinge:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.AngularLimit, true);
                case ConstraintType.Fixed:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.LinearLimit | ConstraintCapabilities.AngularLimit, true);
                case ConstraintType.Slider:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.LinearLimit | ConstraintCapabilities.AngularLimit, true);
                case ConstraintType.BallSocket:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.AngularLimit, false);
                case ConstraintType.ConeTwist:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.AngularLimit | ConstraintCapabilities.AngularMotor, false);
                case ConstraintType.SixDof:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.LinearLimit | ConstraintCapabilities.AngularLimit | ConstraintCapabilities.LinearMotor | ConstraintCapabilities.AngularMotor | ConstraintCapabilities.Spring, false);
                case ConstraintType.Motor:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.LinearMotor | ConstraintCapabilities.AngularMotor, false);
                case ConstraintType.Gear:
                    return new ConstraintDescriptor(type, ConstraintCapabilities.AngularMotor, false);
                default:
                    return new ConstraintDescriptor(ConstraintType.None, ConstraintCapabilities.None, false);
            }
        }
    }
}
