# Xia recruitment and force

Xia's active fraction MA represents recruited fibers. Its target command is
activation a, but instantaneous active force uses MA once, not a times MA.
The Thelen and De Groote fatigable variants now use that interpretation when
the fatigue state is DYNAMIC_XIA. Simple capacity fatigue retains a times MA.
Passive force and damping are unaffected by this change.

Existing Xia simulations will change, especially at submaximal commands.
Initialize the compartments explicitly: a resting muscle has MA=0, MF=0, MR=1.
The historical default MA=1 describes a fully recruited muscle and is retained
for API compatibility. Do not interpret it as a resting initial condition.
At fixed recruitment, changing the command affects future recruitment through
the compartment dynamics; it does not instantaneously recruit fibers twice.
CasADi force evaluation is supported, but Xia time derivatives remain unavailable.

Reference: Xia and Frey Law (2008), doi:10.1016/j.jbiomech.2008.07.013.
Tracking issue: https://github.com/pyomeca/biorbd/issues/402
