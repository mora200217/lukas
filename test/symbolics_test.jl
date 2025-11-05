using Symbolics
using Latexify
@variables x y 

f(x,y) = 2*x + y^2

Dx = Differential(x)


fp = Dx(2 * x * y * sin(x^2))

sol = Symbolics.expand_derivatives(fp)

Latexify.latexify(sol)

@variables t 

Dt = Differential(t)
R = [2t, t^2] 


sol = Symbolics.expand_derivatives.(Dt.(R))





