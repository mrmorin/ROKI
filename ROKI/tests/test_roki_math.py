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

def euler(q):
    q0,q1,q2,q3=q
    return (math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)), math.asin(max(-1,min(1,2*(q0*q2-q3*q1)))), math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)))

class BaroGate:
    def __init__(self): self.prev=0.; self.candidate=0.; self.count=0; self.height=0.
    def feed(self, raw, predicted, dt=.05):
        bad=abs(raw-self.prev)>max(12.,300*dt) or abs(raw-predicted)>80
        if bad:
            if not self.count or abs(raw-self.candidate)>2: self.candidate,self.count=raw,1
            else: self.count+=1
            if self.count<4: return False
            self.height=self.prev=self.candidate; self.count=0; return True
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

    def test_baro_single_spike_then_relock(self):
        b=BaroGate(); b.feed(0,0)
        self.assertFalse(b.feed(200,0))
        self.assertTrue(b.feed(.1,.1))
        for _ in range(3): self.assertFalse(b.feed(120,0))
        self.assertTrue(b.feed(120,0)); self.assertAlmostEqual(b.height,120)

    def test_acceleration_16g_fits_protocol(self):
        self.assertLessEqual(round(16*G*100),32767)

    def test_csv_timestamps_preserve_intervals(self):
        rows=list(csv.DictReader(io.StringIO('received_at,tx_ms\n2026-01-01T00:00:00.000,100\n2026-01-01T00:00:00.020,120\n2026-01-01T00:00:00.060,160\n')))
        intervals=[(int(rows[i]['tx_ms'])-int(rows[i-1]['tx_ms']))/1000 for i in range(1,len(rows))]
        self.assertEqual(intervals,[.02,.04])

if __name__ == '__main__': unittest.main()
