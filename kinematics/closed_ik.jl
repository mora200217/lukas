module SymIK2R
export inverse_kinematics_2R

using Symbolics

"""
    inverse_kinematics_2R(x(t), y(t), L1, L2)

Retorna q1(t), q2(t) simbólicos para un robot 2R.
"""
function inverse_kinematics_2R(x, y, L1, L2)
    @variables t

    # Distancia radial
    D = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)

    # q2(t)
    q2 = acos(D)

    # q1(t)
    q1 = atan(y, x) - atan(L2*sin(q2), L1 + L2*cos(q2))

    return simplify.(q1), simplify.(q2)
end

end # module
