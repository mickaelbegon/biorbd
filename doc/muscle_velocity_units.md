# Muscle shortening speed units

Use `maxShorteningSpeedUnits` in a muscle definition to make the physical scale
explicit. Values are `optimal_lengths_per_second`, `length_units_per_second`,
and `legacy`. The default remains `legacy` for existing files and constructors.

For example, `maxShorteningSpeed 10` with `optimal_lengths_per_second` and
`optimalLength 0.1` represents a maximum fiber speed of 1 length unit/s. The
equivalent absolute setting is `maxShorteningSpeed 1` with
`length_units_per_second`. These scales are shared by force-velocity and damping.

In C++/Python, use Characteristics.setMaxShorteningSpeedUnits() with
OPTIMAL_LENGTHS_PER_SECOND or LENGTH_UNITS_PER_SECOND. The getter reports the
selected convention; HillType.maxShorteningSpeedInLengthUnits() resolves the
absolute scale, including the muscle family in legacy mode.

Legacy retains normalized Thelen speeds, absolute Hill/De Groote speeds, and
historical damping normalization. Migrating a legacy Thelen speed to normalized
units preserves its numeric parameter. Migrating a legacy Hill/De Groote speed
to absolute units preserves its parameter; migrating it to normalized units
requires dividing the absolute parameter by optimalLength. Explicit units also
correct the old damping mismatch, so compare simulations when damping is enabled.
The separate fiber-velocity correction changes pennated-muscle results.

Parameters and geometry must use consistent model length units; angles are in
radians. Values must be positive. Writer::writeModel currently omits muscle
definitions, so retain the source .bioMod when saving these properties.
Tracking issue: https://github.com/pyomeca/biorbd/issues/402
