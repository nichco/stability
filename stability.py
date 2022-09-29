import csdl
import python_csdl_backend

class stability(csdl.Model):
    def initialize(self):
        pass
    def define(self):
        # bracketed search: solve for l from s_h? in order to balance moments
        model = csdl.Model()
        s_h = model.declare_variable('htail_area')
        c_bar = model.declare_variable('mean_aerodynamic_chord')
        s = model.declare_variable('wing_area')
        cl_h = model.declare_variable('cl_h') # horizontal tail lift coefficient
        h_0 = model.declare_variable('h_0') # fraction distance from wing LE to wing aerodynamic center (h_0*chord = dist)
        h = model.declare_variable('h') # fraction distance from wing LE to aircraft CG (h*chord = dist)
        cm = model.declare_variable('cm') # wing and fuselage moment coefficient
        cl = model.declare_variable('cl')
        l = model.declare_variable('l') # distance between wing and tail aerodynamic centers

        y = cm + cl*(h - h_0) - (l/c_bar)*(s_h/s)*cl_h # from Sadraey aircraft design 2012

        model.register_output('y', y)

        solver = self.create_implicit_operation(model)
        solver.declare_state('l', residual='y', bracket=(0, 100)) # solve for distance between aerodynamic centers
        
        s_h = self.declare_variable('htail_area')
        c_bar = self.declare_variable('mean_aerodynamic_chord')
        s = self.declare_variable('wing_area')
        cl_h = self.declare_variable('cl_h') # horizontal tail lift coefficient
        h_0 = self.declare_variable('h_0') # fraction distance from wing LE to wing aerodynamic center (h_0*chord = dist)
        h = self.declare_variable('h') # fraction distance from wing LE to aircraft CG (h*chord = dist)
        cm = self.declare_variable('cm') # wing and fuselage moment coefficient
        cl = self.declare_variable('cl')

        l = solver(s_h,c_bar,s,cl_h,h_0,h,cm,cl)

        # tail volume coefficient
        v_h = (l/c_bar)*(s_h/s)
        self.register_output('tail_volume_coefficient',v_h)


sim = python_csdl_backend.Simulator(stability())
sim.run()

print('l', sim['l'])

