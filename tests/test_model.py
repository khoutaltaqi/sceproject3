import unittest
import numpy as np
from rm_phase3.common import settings
from rm_phase3.model import EKF, motion, tag_model, wrap, cubic
C = settings()


class MathTests(unittest.TestCase):
    def test_lecture_11a_numbers_and_jacobians(self):
        x=np.array([1.,2.,.6,.8,.3]);u=np.array([.2,-.1]);dt=.1
        _,F,G=motion(x,u,dt)
        self.assertAlmostEqual(F[0,2],-.0451713979,places=9)
        self.assertAlmostEqual(F[1,2], .0660268492,places=9)
        eps=1e-6
        num=np.column_stack([(motion(x+np.eye(5)[j]*eps,u,dt)[0]-motion(x-np.eye(5)[j]*eps,u,dt)[0])/(2*eps) for j in range(5)])
        np.testing.assert_allclose(num,F,atol=1e-8)
        numG=np.column_stack([(motion(x,u+np.eye(2)[j]*eps,dt)[0]-motion(x,u-np.eye(2)[j]*eps,dt)[0])/(2*eps) for j in range(2)])
        np.testing.assert_allclose(numG,G,atol=1e-8)

    def test_lecture_11b_position_correction(self):
        e=EKF(C);e.x=np.array([1.,2.,.6,.8,.3]);e.P=np.eye(5)*.1
        e.P[0,3]=e.P[3,0]=.08;e.P[3,3]=.32
        e.Ro=np.diag([.04,.04]);e.odom(.6,.3)
        self.assertAlmostEqual(e.x[0],.95555555556,places=9)

    def test_lecture_12a_jacobian_and_bearing_correction(self):
        x=np.array([1.,.4,.1,.8,.3]);tag=[3.,1.];h,H=tag_model(x,tag)
        self.assertAlmostEqual(h[1],.19145679448,places=9)
        eps=1e-6
        num=np.column_stack([(tag_model(x+np.eye(5)[j]*eps,tag)[0]-tag_model(x-np.eye(5)[j]*eps,tag)[0])/(2*eps) for j in range(5)])
        np.testing.assert_allclose(H,num,atol=1e-8)
        e=EKF(C);e.x=x.copy();e.P=np.diag([.09,.06,.01,.01,.01]);e.P[0,1]=e.P[1,0]=.02
        e.update([.25], h[1:2], H[1:2], np.array([[.01]]),angle=0)
        self.assertAlmostEqual(e.x[1]-.4,-.04560,places=4)

    def test_angle_wrap_cubic_and_rejected_outlier(self):
        self.assertAlmostEqual(wrap(-6.2715),.01168530718,places=9)
        self.assertEqual(cubic(0,10,1.3),(0.,0.))
        self.assertEqual(cubic(10,10,1.3),(1.3,0.))
        self.assertAlmostEqual(cubic(5,10,1.3)[1],.195)
        e=EKF(C);before=e.x.copy();ok,_,_=e.tag([10.,2.],[1.5,0.],np.diag([.01,.01]))
        self.assertFalse(ok);np.testing.assert_array_equal(e.x,before)

    def test_covariance_stays_positive(self):
        e=EKF(C)
        for _ in range(300):
            e.predict([0.,0.],.1);e.odom(.05,.0)
            e.tag(tag_model(e.x,[3.,0.])[0],[3.,0.],np.diag([.01,.01]))
            self.assertGreaterEqual(np.linalg.eigvalsh(e.P).min(),-1e-12)

    def test_saved_update_reproduces_gain_state_and_covariance(self):
        e=EKF(C);e.predict([.1,.02],.1)
        e.tag([1.49,.01],[1.5,0.],np.diag([.0004,.000625]))
        a=e.last_update
        self.assertTrue(a['accepted'])
        P,H,R,K=[np.array(a[k]) for k in ['P_prior','H','R','K']]
        S=H@P@H.T+R
        np.testing.assert_allclose(a['S'],S)
        np.testing.assert_allclose(K,np.linalg.solve(S,H@P).T)
        delta=K@a['innovation']
        np.testing.assert_allclose(delta,a['correction'])
        expected=np.array(a['x_prior'])+delta;expected[2]=wrap(expected[2])
        np.testing.assert_allclose(expected,a['x_post'])
        A=np.eye(5)-K@H
        np.testing.assert_allclose(A@P@A.T+K@R@K.T,a['P_post'],atol=1e-14)

    def test_rejected_update_records_zero_change(self):
        e=EKF(C);before=e.P.copy()
        e.tag([10.,2.],[1.5,0.],np.diag([.01,.01]))
        a=e.last_update
        self.assertFalse(a['accepted']);self.assertIsNone(a['K'])
        np.testing.assert_array_equal(a['correction'],np.zeros(5))
        np.testing.assert_array_equal(a['x_prior'],a['x_post'])
        np.testing.assert_array_equal(a['P_post'],before)


if __name__ == "__main__":
    unittest.main()
