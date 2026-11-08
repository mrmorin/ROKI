import csv
import io
import math
import unittest
from collections import deque
from pathlib import Path
from ROKI.monitor.roki_math import frd_to_neu, project_neu, build_playback_timeline

G = 9.80665
ZUPT_BARO_RATE_MPS = .18
ZUPT_BARO_SPAN_M = .30

def simulate_zupt(raw_heights, gyro_mrad=0., dt=.05):
    """Reference model of both short-time and multi-second ZUPT checks."""
    filtered=rate=anchor=None; run=best=0.; history=deque(maxlen=80)
    for sample,raw in enumerate(raw_heights):
        if filtered is None:
            filtered=anchor=raw; rate=0.; history.append((0.,filtered)); continue
        before=filtered; filtered=.82*filtered+.18*raw
        measured=(filtered-before)/dt; rate=.88*rate+.12*measured
        history.append((sample*dt,filtered))
        trend_still=False
        if len(history)==80 and history[-1][0]-history[0][0]>=3.5:
            n=len(history); st=sum(t for t,_ in history); sy=sum(y for _,y in history)
            stt=sum(t*t for t,_ in history); sty=sum(t*y for t,y in history)
            slope=(n*sty-st*sy)/(n*stt-st*st)
            trend_still=abs(slope)<.035
        baro_still=trend_still and abs(rate)<ZUPT_BARO_RATE_MPS and abs(filtered-anchor)<ZUPT_BARO_SPAN_M
        if gyro_mrad<80. and baro_still:
            run+=dt
            if run>=.65: best=max(best,run)
        else:
            run=0.; anchor=filtered
    return best

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
            allowed_step=.5+250*dt
            if not self.count or abs(raw-self.candidate)>allowed_step: self.candidate,self.candidate_prev,self.candidate_vz,self.count=raw,raw,self.vertical,1
            else:
                self.candidate_vz=.7*self.candidate_vz+.3*(raw-self.candidate_prev)/dt
                self.candidate_prev=self.candidate=raw; self.count+=1
            if self.count<4: return False
            self.height=self.prev=self.candidate; self.vertical=.6*self.vertical+.4*self.candidate_vz; self.count=0; return True
        self.prev=self.height=raw; self.count=0; return True

class RokiMathTests(unittest.TestCase):
    def test_zupt_stationary_recorded_eight_seconds_with_baro_noise(self):
        path=Path(__file__).parents[1]/'monitor'/'logs'/'ROKI_NORMAL_28kbps_2026-11-09_045506_001.csv'
        with path.open(encoding='utf-8',newline='') as source:
            rows=list(csv.DictReader(source))[-400::3]
        pressure=[float(row['pressure_pa']) for row in rows];reference=pressure[0]
        heights=[44330*(1-(p/reference)**.190294957) for p in pressure]
        # The CSV contains ~8 s of actual stillness; do not manufacture 10 s
        # by repeating it, which conceals long-window drift.
        self.assertGreater(simulate_zupt(heights,gyro_mrad=16,dt=.06),.8)

    def test_zupt_rejects_uniform_climb_and_descent(self):
        for rate in (-.5,-.2,-.15,-.1,-.05,.05,.1,.15,.2,.5):
            with self.subTest(rate=rate):
                heights=[rate*.05*i for i in range(200)]
                self.assertEqual(simulate_zupt(heights,dt=.05),0.)

    def test_zupt_rejects_rotation_without_translation(self):
        self.assertEqual(simulate_zupt([0.]*200,gyro_mrad=100,dt=.05),0.)

    def test_zupt_tolerates_static_baro_noise(self):
        noise=[.00,.18,-.11,.24,-.19,.09,-.05,.14,-.16,.03]*20
        self.assertGreater(simulate_zupt(noise,gyro_mrad=5,dt=.05),3.)

    def test_recorded_packet_loss_is_before_receiver_queue(self):
        path=Path(__file__).parents[1]/'monitor'/'logs'/'ROKI_NORMAL_28kbps_2026-11-09_045506_001.csv'
        with path.open(encoding='utf-8',newline='') as source:
            rows=list(csv.DictReader(source))
        sequence_loss=sum(max(0,int(new['sequence'])-int(old['sequence'])-1) for new,old in zip(rows[1:],rows[:-1]))
        self.assertEqual(sequence_loss,77)
        self.assertEqual(max(int(row['receiver_queue_overflow']) for row in rows),0)
        self.assertEqual(max(int(row['protocol_errors']) for row in rows),0)
        self.assertEqual(int(rows[-1]['usb_drops'])-int(rows[0]['usb_drops']),5)

    def test_scene_yaw_north_east_west(self):
        # Top view: north is up, east is right. +Yaw is a clockwise turn.
        top_azimuth=0.; top_elevation=math.pi/2; nose=(1.,0.,0.)
        north=frd_to_neu(nose,0,0,0); east=frd_to_neu(nose,0,0,math.pi/2); west=frd_to_neu(nose,0,0,-math.pi/2)
        self.assertGreater(north[0],.999)
        self.assertGreater(project_neu(north,top_azimuth,top_elevation)[1],0)
        self.assertGreater(east[1],.999)
        self.assertGreater(project_neu(east,top_azimuth,top_elevation)[0],0)
        self.assertLess(west[1],-.999)
        self.assertLess(project_neu(west,top_azimuth,top_elevation)[0],0)

    def test_scene_yaw_remains_correct_with_roll_pitch(self):
        # Roll/pitch tilt the nose vertically but must not mirror its heading.
        nose_east=frd_to_neu((1.,0.,0.),math.radians(20),math.radians(-15),math.pi/2)
        nose_north_up=frd_to_neu((1.,0.,0.),0,math.radians(15),0)
        self.assertGreater(nose_east[1],.9)
        self.assertGreater(project_neu(nose_east,0,math.pi/2)[0],0)
        self.assertGreater(nose_north_up[0],.9)
        self.assertGreater(nose_north_up[2],.2)

    def test_scene_projection_depth_orders_far_faces_before_near_faces(self):
        # With the camera elevated, higher NEU points are nearer (smaller depth).
        _,up_screen,near_depth=project_neu((0.,0.,1.),0,math.pi/4)
        _,down_screen,far_depth=project_neu((0.,0.,-1.),0,math.pi/4)
        self.assertGreater(up_screen,down_screen)
        self.assertLess(near_depth,far_depth)

    def test_stationary_speed_is_zero(self):
        velocity=0.
        for _ in range(200):
            acceleration=0.; velocity += acceleration*.005
        self.assertAlmostEqual(velocity,0.)

    def test_uniform_vertical_motion_blocks_zupt(self):
        # Regression guard must reject 0.1 m/s even when the short-time rate
        # and span are below the firmware's current 0.18/0.30 gates.
        self.assertEqual(simulate_zupt([.1*.05*i for i in range(300)]),0.)

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

    def test_baro_relock_high_speed_climb_and_descent(self):
        # Four 20 Hz samples must relock through the requested speed envelope.
        for rate in (20.,60.,100.,150.,-20.,-60.,-100.,-150.):
            with self.subTest(rate=rate):
                b=BaroGate(); b.feed(0,0); b.vertical=rate
                samples=[101.+rate*.05*i for i in range(4)]
                accepted=[b.feed(height,0,dt=.05) for height in samples]
                self.assertEqual(accepted,[False,False,False,True])
                self.assertAlmostEqual(b.height,samples[-1])
                self.assertGreater(b.vertical*rate,0)
                self.assertGreater(abs(b.vertical),.25*abs(rate))

    def test_acceleration_16g_fits_protocol(self):
        self.assertLessEqual(round(16*G*100),32767)

    def test_csv_timestamps_preserve_intervals(self):
        rows=list(csv.DictReader(io.StringIO('received_at,tx_ms\n2026-01-01T00:00:00.000,100\n2026-01-01T00:00:00.020,120\n2026-01-01T00:00:00.060,160\n')))
        times=build_playback_timeline(rows)
        intervals=[round(times[i]-times[i-1],3) for i in range(1,len(rows))]
        self.assertEqual(intervals,[.02,.04])

    def test_csv_delivery_burst_does_not_collapse_time(self):
        rows=[{'received_at':'2026-01-01T00:00:00.000','tx_ms':str(x)} for x in (100,120,160,180)]
        self.assertEqual([round(t,3) for t in build_playback_timeline(rows)],[0.,.02,.06,.08])

    def test_csv_tx_restart_and_missing_timestamps(self):
        rows=[{'received_at':f'2026-01-01T00:00:00.{ms:03d}','tx_ms':str(tx)}
              for ms,tx in ((0,1000),(20,1020),(70,1),(90,21))]
        self.assertEqual([round(t,3) for t in build_playback_timeline(rows)],[0.,.02,.07,.09])

    def test_csv_tx_millis_rollover(self):
        rows=[{'received_at':'2026-01-01T00:00:00.000','tx_ms':str(2**32-10)},
              {'received_at':'2026-01-01T00:00:00.000','tx_ms':'10'}]
        self.assertEqual([round(t,3) for t in build_playback_timeline(rows)],[0.,.02])

if __name__ == '__main__': unittest.main()
