# Position or velocity type for Novatel BESTPOS and BESTUTM messages
# 
# The possible values are taken from table 47 of OEM4 Family USER MANUAL -
# VOLUME 2 Command and Log Reference, repeated below for convenience.
#
# Type (binary) | Type (ASCII)     | Description
# --------------|------------------|------------------------------------------------------------
# 0             | NONE             | No solution
# 1             | FIXEDPOS         | Position has been fixed by the FIX POSITION command
# 2             | FIXEDHEIGHT      | Position has been fixed by the FIX HEIGHT/AUTO command
# 3-7           | Reserved         | 
# 8             | DOPPLER_VELOCITY | Velocity computed using instantaneous Doppler
# 9-15          | Reserved         | 
# 16            | SINGLE           | Single point position
# 17            | PSRDIFF          | Pseudorange differential solution
# 18            | WAAS             | Solution calculated using corrections from an SBAS
# 19            | PROPAGATED       | Propagated by a Kalman filter without new observations
# 20            | OMNISTAR         | OmniSTAR VBS position (L1 sub-meter)
# 21-31         | Reserved         | 
# 32            | L1_FLOAT         | Floating L1 ambiguity solution
# 33            | IONOFREE_FLOAT   | Floating ionospheric-free ambiguity solution
# 34            | NARROW_FLOAT     | Floating narrow-lane ambiguity solution
# 48            | L1_INT           | Integer L1 ambiguity solution
# 49            | WIDE_INT         | Integer wide-lane ambiguity solution
# 50            | NARROW_INT       | Integer narrow-lane ambiguity solution
# 51            | RTK_DIRECT_INS   | RTK status where the RTK filter is directly initialized
#               |                  |   from the INS # filter
# 52-56         | INS calculated   | 
#               |   position types | 
# 64            | OMNISTAR_HP      | OmniSTAR HP position (L1/L2 decimeter)
# 65            | OMNISTAR_XP      | OmniSTAR XP position
# 66            | CDGPS            | Position solution using CDGPS correction

Header header
uint16 type_binary
string type_ascii