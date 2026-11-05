# Fixed-point numeric contract

`fix` stores a signed 64-bit raw integer with 16 fractional bits. One raw unit is
`1 / 65536`. `long.MinValue` is reserved for `fix.NaN`; finite raw values are
`[long.MinValue + 1, long.MaxValue]`.

## Arithmetic

| Operation | Policy |
| --- | --- |
| Integer construction | Scale by 65536; return NaN when the result is outside the finite range. |
| Add/subtract | Exact raw arithmetic; return NaN on overflow. |
| Multiply | Compute the full product, then shift right by 16. Discarded bits round toward negative infinity, including negative sub-unit products. |
| Divide | Compute the full scaled numerator; truncate the quotient toward zero. Division by zero returns NaN. |
| Remainder | Use the dividend's sign. Zero divisors and invalid operands return NaN. |
| Square root | Return the integer floor of `sqrt(raw << 16)`. Negative input returns NaN. |
| Left shift | Mask the count with 63, matching a C# `long` shift; return NaN when the shifted value is outside the finite range. |
| Right shift | Arithmetic shift with the same count masking. |

Invalid operands propagate through these operations and through `math.add`,
`math.sub`, and `math.mod`. No arithmetic operation allocates managed memory.

Modern .NET uses `Int128` for multiplication, division, and square root. Other
profiles use integer-only portable implementations with the same raw results.
The portable implementation uses a split 128-bit product, quotient/remainder
division, and a digit-by-digit integer square root. It does not use floating
point or `BigInteger` in the runtime library.

## Rounding and vectors

- `trunc` rounds toward zero, including exact negative integers.
- `floor` rounds toward negative infinity; `ceil` toward positive infinity.
- `round` selects the nearest integer, with midpoint ties away from zero.
- `frac` preserves the existing API's magnitude-of-fraction convention:
  `abs(x - trunc(x))`. For example, `frac(-1.25)` is `0.25`.
- Invalid inputs and unrepresentable rounded results return NaN.
- Constructing `fix2`, `fix3`, or `fix4` from a `fix` scalar copies its raw value
  to every component, including fractional values and NaN.
- Vector arithmetic operators use the scalar overflow and invalid-value policy
  for each component.

These policies are explicit API choices. In particular, callers should not assume
Unity.Mathematics' midpoint or fractional-part conventions.

`Geometry.Angle` keeps its intermediate values in fixed point. A zero or
below-resolution denominator returns zero; an invalid denominator or dot product
returns NaN. Its epsilon is one representable raw unit.

## Verification

The test executable includes boundary cases, deterministic input corpora, exact
`BigInteger` arithmetic oracles, square-root inequalities, decimal rounding
checks, vector splats, angle edge cases, and allocation checks. `BigInteger` is a
test-only dependency supplied by .NET.

Run the normal and portable backends using a .NET 9 SDK:

```bash
dotnet run --project Maphy.Net.Tests~/Maphy.Net.Tests.csproj -c Release
dotnet run --project Maphy.Net.Tests~/Maphy.Net.Tests.csproj -c Release -p:MaphyForcePortable=true
```

`MaphyForcePortable=true` defines `MAPHY_FORCE_PORTABLE` in the core project. It
selects the same arithmetic implementation used by profiles without `Int128`.
Compare the printed corpus hashes as well as the test exit codes. CI runs both
backends and checks their hashes.

This verifies backend agreement on .NET. It does not certify Unity Mono or
IL2CPP execution. Floating-point conversions, remaining vector/matrix APIs,
trigonometric accuracy, and cross-platform physics replay still need the next
validation stages in [ROADMAP.md](ROADMAP.md).
