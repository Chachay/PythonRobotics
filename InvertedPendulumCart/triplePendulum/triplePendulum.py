"""
Triple Pendulum Cart Simulation

Author: Chachay 
"""
import Box2D as b2
import numpy as np
import wx

FPS = 50

class TriplePendulumCartBox2D(object):
    def __init__(self):
        ARM_LENGTH   = 2.2
        NUM_OF_ARMS  = 3
        CART_INIT_X  = 0
        CART_TRANS_LIMIT = 20

        self.world = b2.b2World(gravity=(0, -9.8))

        self.dt = 1.0/FPS
        self.velocityIterations = 8 
        self.positionIterations = 3
        self.static_body  = self.world.CreateStaticBody(position=(0, -NUM_OF_ARMS*ARM_LENGTH + 2),
                                                        shapes = b2.b2PolygonShape(box=(1, 0.5) ))

        arm  = b2.b2FixtureDef(categoryBits=0x0002, maskBits=0x0004, 
                               shape=b2.b2PolygonShape(box=(0.3, ARM_LENGTH)), density=1, friction=0.1)
        cart = b2.b2FixtureDef(categoryBits=0x0002, maskBits=0x0004, 
                               shape=b2.b2PolygonShape(box=(2  , 1)), density=1, friction=0.1)

        self.cart   = self.world.CreateDynamicBody(fixtures=cart,
                                                   position=(CART_INIT_X, 0), angle=0)
        self.pendulum = []
        for i in range(NUM_OF_ARMS):
            self.pendulum.append(
                self.world.CreateDynamicBody(fixtures=arm, position=(CART_INIT_X, -ARM_LENGTH * (2*i + 1)), angle=0)
            )
        self.joint = []
        self.joint.append(
            self.world.CreateRevoluteJoint(bodyA=self.cart, bodyB=self.pendulum[0], anchor=(CART_INIT_X, 0))
        )
        for i in range(1, NUM_OF_ARMS):
            self.joint.append(
                self.world.CreateRevoluteJoint(bodyA=self.pendulum[i-1], bodyB=self.pendulum[i],
                                               anchor=(CART_INIT_X, -ARM_LENGTH*i*2))
            )
        self.slider = self.world.CreatePrismaticJoint(
            bodyA = self.static_body,
            bodyB = self.cart,
            axis  = (1, 0.0),
            enableLimit = True,
            enableMotor = True,
            motorSpeed = 0.0,
            maxMotorForce = 1.0,
            lowerTranslation = -CART_TRANS_LIMIT,
            upperTranslation =  CART_TRANS_LIMIT
        )

    def Render(self, X_offset, Y_offset, PixelPerMeter):
        # Return Cart Shape
        shape = self.cart.fixtures[0].shape
        vertices = [(self.cart.transform * v) * PixelPerMeter for v in shape.vertices]

        yield "Cart", [(X_offset+ int(v[0]), int(Y_offset- v[1])) for v in vertices]

        for body in self.pendulum:
            shape = body.fixtures[0].shape
            vertices = [(body.transform * v) * PixelPerMeter for v in shape.vertices]
            yield "Pendulum", [(X_offset + int(v[0]), int(Y_offset - v[1])) for v in vertices]
        
    def Command(self, force, speed):
        self.slider.maxMotorForce = np.abs(force)
        self.slider.motorSpeed    = speed

    def Step(self):
        self.world.Step(self.dt, self.velocityIterations, self.positionIterations)

class SwingUpController(object):
    def __init__(self):
        super(SwingUpController, self).__init__()
        self.u_prev = 0.0

    def IntegratedController(self, y_prev):
        x_target, y_target = self.TrajectoryGenerator()
        u_ff = self.FeedforwardController(y_target)

        x_est = self.Observer(self.u_prev, y_prev)
        u_fb = self.FeedbackController(x_target - x_est)

        u = u_ff + u_fb
        self.u_prev = u

        return u

    def TrajectoryGenerator(self):
        pass

    def FeedbackController(self, x_diff):
        pass

    def FeedforwardController(self, y_target):
        pass

    def Observer(self, u_prev, y_prev):
        pass


class APPWINDOW(wx.Frame):
    W = 1024
    H = 640
    PixelYOffset = 240
    PixelXOffset = 512
    PixelPerMeter = 20.0  # pixels per meter

    def __init__(self, parent=None, id=-1, title=None):
        super(APPWINDOW, self).__init__(parent, id, title)
        self.SetSize(self.W, self.H)
        self.timer = wx.Timer(self)

        self.model = TriplePendulumCartBox2D()

        self.Bind(wx.EVT_TIMER, self.OnTimer)
        self.Bind(wx.EVT_CLOSE, self.CloseWindow)

        self.i = 0

        self.timer.Start(1.0/FPS*1000)

    def CloseWindow(self, event):
        wx.Exit()
    
    def draw(self):
        bdc = wx.BufferedDC(wx.ClientDC(self))
        gcdc = wx.GCDC(bdc)
        gcdc.Clear()

        gcdc.SetPen(wx.Pen('white'))
        gcdc.SetBrush(wx.Brush('white'))
        gcdc.DrawRectangle(0,0,self.W,self.H)

        # Draw Objects
        for obj, vertices in self.model.Render(self.PixelXOffset, self.PixelYOffset, self.PixelPerMeter):
            if obj == "Cart":
                gcdc.SetPen(wx.Pen(wx.Colour(50,50,50)))
                gcdc.SetBrush(wx.Brush(wx.Colour(112,146,190)))
            else: # Pendulum
                gcdc.SetPen(wx.Pen(wx.Colour(50,50,50)))
                gcdc.SetBrush(wx.Brush(wx.Colour(253, 228, 177)))
            gcdc.DrawPolygon(vertices)

    def OnTimer(self, event):
        speed = np.sin(self.i*2.0*np.pi*0.1/FPS) * 10
        self.model.Command(speed*40, speed)

        self.model.Step()

        self.draw()
        self.i += 1

if __name__ == '__main__':
    app = wx.App()
    w = APPWINDOW(title='TripePendulumCart')

    w.Center()
    w.Show()
    app.MainLoop()
        