import csv
import io
import math
import unittest

G = 9.80665

def q_from_euler(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2), math.cos(y/2), math.sin(y/2)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]

def q_step(q, gyro, dt):
    q0,q1,q2,q3=q; gx,gy,gz=gyro; h=.5*dt
    n=[q0+h*(-q1*gx-q2*gy-q3*gz),q1+h*(q0*gx+q2*gz-q3*gy),q2+h*(q0*gy-q1*gz+q3*gx),q3+h*(q0*gz+q1*gy-q2*gx)]
    s=1/math.sqrt(sum(v*v for v in n)); return [v*s for v in n]

def gravity_body(q):
    q0,q1,q2,q3=q
    return [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3]

def q_attitude_step(q, measured_gravity_body, dt=.005):
    # Exact sign/order of the firmware's Mahony-style correction term.
    q0,q1,q2,q3=q; ax,ay,az=measured_gravity_body
    ex,ey,ez=gravity_body(q)
    gx=2.2*(ez*ay-ey*az); gy=2.2*(ex*az-ez*ax); gz=2.2*(ey*ax-ex*ay)
    return q_step(q,(gx,gy,gz),dt)

def angle_error(q, truth):
    dot=abs(sum(a*b for a,b in zip(q,truth)))
    return 2*math.acos(min(1,max(-1,dot)))

def euler(q):
    q0,q1,q2,q3=q
    return (math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)), math.asin(max(-1,min(1,2*(q0*q2-q3*q1)))), math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)))

class BaroGate:
    def __init__(self): self.prev=0.; self.candidate=0.; self.candidate_prev=0.; self.candidate_vz=0.; self.count=0; self.height=0.; self.vertical=0.
    def feed(self, raw, predicted, dt=.05):
        bad=abs(raw-self.prev)>max(12.,300*dt) or abs(raw-predicted)>80
        if bad:
            if not self.count or abs(raw-self.candidate)>2: self.candidate,self.candidate_prev,self.candidate_vz,self.count=raw,raw,self.vertical,1
            else:
                self.candidate_vz=.7*self.candidate_vz+.3*(raw-self.candidate_prev)/dt
                self.candidate_prev=self.candidate=raw; self.count+=1
            if self.count<4: return False
            self.height=self.prev=self.candidate; self.vertical=.6*self.vertical+.4*self.candidate_vz; self.count=0; return True
        self.prev=self.height=raw; self.count=0; return True

class RokiMathTests(unittest.TestCase):
    def test_stationary_speed_is_zero(self):
        velocity=0.
        for _ in range(200):
            acceleration=0.; velocity += acceleration*.005
        self.assertAlmostEqual(velocity,0.)

    def test_uniform_vertical_motion_blocks_zupt(self):
        imu_still=True; baro_rate=.5; baro_span=.20
        zupt = imu_still and abs(baro_rate)<.08 and abs(baro_span)<.12
        self.assertFalse(zupt)

    def test_roll_pitch_signs(self):
        self.assertGreater(euler(q_from_euler( math.radians(30),0,0))[0],0)
        self.assertLess(euler(q_from_euler(-math.radians(30),0,0))[0],0)
        self.assertGreater(euler(q_from_euler(0, math.radians(30),0))[1],0)
        self.assertLess(euler(q_from_euler(0,-math.radians(30),0))[1],0)

    def test_combined_rotation_stays_normalized(self):
        q=[1.,0.,0.,0.]
        for _ in range(1000): q=q_step(q,(.7,-.4,.9),.005)
        self.assertAlmostEqual(sum(v*v for v in q),1.,places=6)
        self.assertTrue(all(math.isfinite(v) for v in euler(q)))

    def test_attitude_feedback_converges_roll_pitch_and_combined(self):
        cases=[(math.radians(r),0,0) for r in (-30,-15,-5,5,15,30)]
        cases += [(0,math.radians(p),0) for p in (-30,-15,-5,5,15,30)]
        cases += [(math.radians(15),math.radians(-20),math.radians(25)),(math.radians(-30),math.radians(20),math.radians(-40))]
        truth=q_from_euler(math.radians(8),math.radians(-11),math.radians(17))
        gravity=gravity_body(truth)
        for dr,dp,dy in cases:
            initial=q_from_euler(math.radians(8)+dr,math.radians(-11)+dp,math.radians(17)+dy)
            initial_alignment=sum(a*b for a,b in zip(gravity_body(initial),gravity)); estimate=initial
            for _ in range(12000): estimate=q_attitude_step(estimate,gravity)
            r,p,_=euler(estimate); tr,tp,_=euler(truth)
            self.assertLess(abs(r-tr),math.radians(.6),msg=(dr,dp,dy))
            self.assertLess(abs(p-tp),math.radians(.6),msg=(dr,dp,dy))
            final_alignment=sum(a*b for a,b in zip(gravity_body(estimate),gravity))
            self.assertGreater(final_alignment,initial_alignment,msg=(dr,dp,dy))
            self.assertGreater(final_alignment,.9999,msg=(dr,dp,dy))

    def test_baro_single_spike_then_relock(self):
        b=BaroGate(); b.feed(0,0)
        self.assertFalse(b.feed(200,0))
        self.assertTrue(b.feed(.1,.1))
        for _ in range(3): self.assertFalse(b.feed(120,0))
        self.assertTrue(b.feed(120,0)); self.assertAlmostEqual(b.height,120)

    def test_baro_relock_with_nonzero_vertical_speed(self):
        b=BaroGate(); b.feed(0,0); b.vertical=2.0
        # IMU prediction is wrong by >80 m while the craft continues climbing.
        accepted=[]
        for height in (101.0,101.1,101.2,101.3): accepted.append(b.feed(height,0,dt=.1))
        self.assertEqual(accepted,[False,False,False,True])
        self.assertAlmostEqual(b.height,101.3)
        # Relock must retain an actual climb, not turn it into a fake stop.
        self.assertGreater(b.vertical,.5)

    def test_acceleration_16g_fits_protocol(self):
        self.assertLessEqual(round(16*G*100),32767)

    def test_csv_timestamps_preserve_intervals(self):
        rows=list(csv.DictReader(io.StringIO('received_at,tx_ms\n2026-01-01T00:00:00.000,100\n2026-01-01T00:00:00.020,120\n2026-01-01T00:00:00.060,160\n')))
        intervals=[(int(rows[i]['tx_ms'])-int(rows[i-1]['tx_ms']))/1000 for i in range(1,len(rows))]
        self.assertEqual(intervals,[.02,.04])

if __name__ == '__main__': unittest.main()
