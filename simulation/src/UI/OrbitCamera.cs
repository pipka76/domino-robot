using Godot;

namespace DominoSim;

public partial class OrbitCamera : Camera3D
{
    public Vector3 Target = new(0.625f, 0, 0.84f);
    public Node3D Follow;
    public bool FollowEnabled;
    float _yaw = 0f, _pitch = 1.0f, _dist = 2.1f;
    bool _top = true;
    Vector3 _home;

    public override void _Ready()
    {
        _home = Target;
        Fov = 45;
        Near = 0.01f;
        Apply();
    }

    public void ToggleTop() { _top = !_top; Apply(); }
    public void ToggleFollow() { FollowEnabled = !FollowEnabled; if (!FollowEnabled) Target = _home; Apply(); }

    public override void _Process(double delta)
    {
        if (FollowEnabled && Follow != null)
        {
            Target = Target.Lerp(new Vector3(Follow.GlobalPosition.X, 0, Follow.GlobalPosition.Z), 0.1f);
            Apply();
        }
    }

    public override void _UnhandledInput(InputEvent e)
    {
        if (e is InputEventMouseMotion mm && (mm.ButtonMask & MouseButtonMask.Right) != 0)
        {
            _top = false;
            _yaw -= mm.Relative.X * 0.006f;
            _pitch = Mathf.Clamp(_pitch + mm.Relative.Y * 0.006f, 0.15f, 1.5f);
            Apply();
        }
        else if (e is InputEventMouseButton mb && mb.Pressed)
        {
            if (mb.ButtonIndex == MouseButton.WheelUp) { _dist = Mathf.Max(0.3f, _dist * 0.9f); Apply(); }
            if (mb.ButtonIndex == MouseButton.WheelDown) { _dist = Mathf.Min(6f, _dist * 1.1f); Apply(); }
        }
    }

    void Apply()
    {
        if (_top)
        {
            Projection = ProjectionType.Orthogonal;
            Size = FollowEnabled ? _dist * 0.35f : 1.85f;
            LookAtFromPosition(Target + new Vector3(0, 3, 0), Target, Vector3.Forward);
        }
        else
        {
            Projection = ProjectionType.Perspective;
            // yaw 0 = camera at +z side (bottom of the field) looking toward -z.
            var off = new Vector3(Mathf.Sin(_yaw) * Mathf.Cos(_pitch), Mathf.Sin(_pitch), Mathf.Cos(_yaw) * Mathf.Cos(_pitch)) * _dist;
            LookAtFromPosition(Target + off, Target, Vector3.Up);
        }
    }
}
