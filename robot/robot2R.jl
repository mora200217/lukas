module Robot
export Robot2R

using Unitful

# Robot2R parametrizado para aceptar Float64 o Unitful.Quantity
    struct Robot2R{T<:Unitful.Quantity}
        L1::T
        L2::T
        M1::T
        M2::T
        base_offset_x::T
    end
end
