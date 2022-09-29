import csdl
import python_csdl_backend

class stability(csdl.Model):
    def initialize(self):
        pass
    def define(self):

        # tail volume coefficient
        # v_h = (l/c_bar)*(s_h/s)
        # self.register_output('tail_volume_coefficient',v_h)

        # bracketed search: solve for l from s_h? in order to balance moments
        model = csdl.Model()
        s_h = model.declare_variable('htail_area')
        c_bar = model.declare_variable('mean_aerodynamic_chord')
        s = model.declare_variable('wing_area')
        cl_h = model.declare_variable('cl_h') # horizontal tail lift coefficient
        h_0 = model.declare_variable('h_0') # fraction distance from wing LE to wing aerodynamic center (h_0*chord = dist)
        h = model.declare_variable('wing_area') # fraction distance from wing LE to aircraft CG (h*chord = dist)
        cm = model.declare_variable('cm') # wing and fuselage moment coefficient
        cl = model.declare_variable('cl')
        l = model.declare_variable('l') # distance between wing and tail aerodynamic centers

        y = cm + cl*(h - h_0) - (l/c_bar)*(s_h/s)*cl_h

        model.register_output('y', y)

        solver = self.create_implicit_operation(model)
        solver.declare_state('l', residual='y', bracket=(0, 100)) # solve for distance between aerodynamic centers
        
        s_h = self.declare_variable('htail_area')
        c_bar = self.declare_variable('mean_aerodynamic_chord')
        s = self.declare_variable('wing_area')
        cl_h = self.declare_variable('cl_h') # horizontal tail lift coefficient
        h_0 = self.declare_variable('h_0') # fraction distance from wing LE to wing aerodynamic center (h_0*chord = dist)
        h = self.declare_variable('wing_area') # fraction distance from wing LE to aircraft CG (h*chord = dist)
        cm = self.declare_variable('cm') # wing and fuselage moment coefficient
        cl = self.declare_variable('cl')

        l = solver(s_h,c_bar,s,cl_h,h_0,h,cm,cl)

        # self.register_output('l',1*l)
        
class ExampleBracketedScalar(csdl.Model):

    def define(self):
        model = csdl.Model()
        a = model.declare_variable('a')
        b = model.declare_variable('b')
        c = model.declare_variable('c')
        x = model.declare_variable('x')
        y = a * x**2 + b * x + c
        model.register_output('y', y)

        solve_quadratic = self.create_implicit_operation(model)
        solve_quadratic.declare_state('x', residual='y', bracket=(0, 2))

        a = self.declare_variable('a', val=1)
        b = self.declare_variable('b', val=-4)
        c = self.declare_variable('c', val=3)
        x = solve_quadratic(a, b, c)


sim = python_csdl_backend.Simulator(ExampleBracketedScalar())
sim.run()

# print('l', sim['l'])

