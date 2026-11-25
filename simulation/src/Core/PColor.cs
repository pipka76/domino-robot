using Godot;

namespace DominoSim;

public enum PColor { None, Red, Green, Blue, Yellow, Black }

public static class PColors
{
    public static PColor Parse(char c) => char.ToUpperInvariant(c) switch
    {
        'R' => PColor.Red,
        'G' => PColor.Green,
        'B' => PColor.Blue,
        'Y' => PColor.Yellow,
        'K' => PColor.Black,
        _ => PColor.None,
    };

    public static char Letter(PColor c) => c switch
    {
        PColor.Red => 'R',
        PColor.Green => 'G',
        PColor.Blue => 'B',
        PColor.Yellow => 'Y',
        PColor.Black => 'K',
        _ => '-',
    };

    public static Color ToColor(PColor c) => c switch
    {
        PColor.Red => new Color(0.85f, 0.10f, 0.10f),
        PColor.Green => new Color(0.10f, 0.65f, 0.20f),
        PColor.Blue => new Color(0.10f, 0.25f, 0.85f),
        PColor.Yellow => new Color(0.95f, 0.82f, 0.10f),
        PColor.Black => new Color(0.06f, 0.06f, 0.06f),
        _ => new Color(0.5f, 0.5f, 0.5f),
    };
}
