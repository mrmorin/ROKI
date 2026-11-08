import bisect, csv, json, math, queue, struct, threading, time
from collections import deque
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
from serial.tools import list_ports

MODES={"NORMAL — 28 кбит/с":"NORMAL","RESERVE — 40 кбит/с":"RESERVE","STRESS — 60 кбит/с":"STRESS"}
QNH_PA=101794.06  # Calibrated at known ground elevation: 175 m.
APP_DIR=Path(__file__).resolve().parent
SETTINGS_PATH=APP_DIR/'roki_settings.json'
CAL_POSES=[
 ('Шаг 1 из 6','Плашмя: микросхема LSM6DS3 сверху.'),
 ('Шаг 2 из 6','Плашмя: микросхема LSM6DS3 снизу.'),
 ('Шаг 3 из 6','Поставьте на хвост: нос Type-C смотрит строго вверх.'),
 ('Шаг 4 из 6','Поставьте на нос: разъём Type-C смотрит строго вниз.'),
 ('Шаг 5 из 6','Нос Type-C от себя. Поднимите правый длинный край платы вверх.'),
 ('Шаг 6 из 6','Нос Type-C от себя. Опустите правый длинный край платы вниз.'),
]

def frd_to_neu(point,roll,pitch,yaw):
 """Rotate a body-FRD vector into world N/E/Up using telemetry RPY."""
 x,y,z=point;cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
 north=cp*cy*x+(sr*sp*cy-cr*sy)*y+(cr*sp*cy+sr*sy)*z
 east=cp*sy*x+(sr*sp*sy+cr*cy)*y+(cr*sp*sy-sr*cy)*z
 down=-sp*x+sr*cp*y+cr*cp*z
 return north,east,-down

def project_neu(point,azimuth,elevation):
 """Return screen-right, screen-up and camera depth for world N/E/Up."""
 north,east,up=point;right=math.cos(azimuth)*east-math.sin(azimuth)*north;depth=math.sin(azimuth)*east+math.cos(azimuth)*north
 # Positive north is screen-up in the top view.  Smaller depth is nearer.
 vertical=math.cos(elevation)*up+math.sin(elevation)*depth
 camera_depth=math.cos(elevation)*depth-math.sin(elevation)*up
 return right,vertical,camera_depth

class App(tk.Tk):
 def __init__(self):
  super().__init__();self.title("ROKI — испытание ESP-NOW LR");self.geometry("1280x820");self.minsize(1060,680);self.configure(bg='#16110d');style=ttk.Style(self);style.theme_use('clam');style.configure('.',background='#16110d',foreground='#f1e6d5',fieldbackground='#24180f',bordercolor='#754417',lightcolor='#754417',darkcolor='#0b0907');style.configure('TFrame',background='#16110d');style.configure('TLabel',background='#16110d',foreground='#f1e6d5');style.configure('TLabelframe',background='#16110d',bordercolor='#754417',relief='solid');style.configure('TLabelframe.Label',background='#16110d',foreground='#f5a23b',font=('Segoe UI',10,'bold'));style.configure('TButton',background='#713c10',foreground='#fff0d5',bordercolor='#d78225',padding=(8,4));style.map('TButton',background=[('active','#a65d18'),('pressed','#4b260c')]);style.configure('TCombobox',fieldbackground='#24180f',background='#713c10',foreground='#fff0d5',arrowcolor='#f5a23b');style.configure('Horizontal.TScale',background='#16110d',troughcolor='#382312',sliderlength=15);style.configure('TNotebook',background='#16110d',borderwidth=0);style.configure('TNotebook.Tab',background='#24180f',foreground='#d7c5aa',padding=(12,6));style.map('TNotebook.Tab',background=[('selected','#713c10')],foreground=[('selected','#fff0d5')]);self.q=queue.Queue();self.ser=None;self.record=None;self.writer=None;self.record_path=None;self.record_stats={};self.run=0;self.rssi=deque(maxlen=3000);self.altitude_history=deque(maxlen=3000);self.pps_history=deque(maxlen=90);self.g_history=deque(maxlen=3000);self.speed_history=deque(maxlen=3000);self.last=time.monotonic();self.last_flush=time.monotonic();self.next_render=0;self.stats={};self.imu_last=None;self.imu_rendered=None;self.roll=self.pitch=self.yaw=0.;self.vehicle_mesh=self.load_vehicle_mesh();self.cal_running=False;self.cal_seen_active=False;self.cal_step=0;self.cal_phase='idle';self.cal_start_attempts=0;self.cal_start_at=0.;self.cal_text=tk.StringVar(value='Нажмите «НАЧАТЬ КАЛИБРОВКУ». Затем программа покажет первое положение.');self.qnh_pa=QNH_PA;self.current_pressure=0;self.baro_ref=tk.StringVar(value='175');self.baro_status=tk.StringVar(value='QNH —');self.baro_pressure=tk.StringVar(value='Давление — Pa');self.baro_calibrated=False;self.mass=tk.StringVar(value='');self.load_settings();self.play_rows=[];self.play_times=[];self.play_index=-1;self.play_time=0.;self.playing=False;self.playback_active=False;self.play_clock=0.;self.play_speed=tk.StringVar(value='1×');self.play_pos=tk.DoubleVar(value=0.);self.play_seeking=False;self.play_last_second=-1;self.play_name=tk.StringVar(value='Лог не загружен');self.play_time_text=tk.StringVar(value='00:00 / 00:00')
  self.trajectory=[];self.trajectory_position=[0.,0.,0.];self.trajectory_velocity=[0.,0.,0.];self.trajectory_accel=[0.,0.,0.];self.trajectory_at=0.;self.trajectory_tx_ms=None;self.trajectory_generation=0;self.rx_queue_overflow=0;self.usb_drops=0;self.protocol_errors=0;self.scene_azimuth=math.radians(35);self.scene_elevation=math.radians(22);self.scene_zoom=1.;self.scene_pan=[0.,0.];self.scene_drag=None
  top=ttk.Frame(self,padding=8);top.pack(fill='x');self.port=tk.StringVar(value='COM9');ttk.Combobox(top,textvariable=self.port,values=[p.device for p in list_ports.comports()],width=8).pack(side='left');ttk.Button(top,text='Подключить',command=self.connect).pack(side='left',padx=4)
  self.mode=tk.StringVar(value=list(MODES)[0]);ttk.Combobox(top,textvariable=self.mode,values=list(MODES),state='readonly',width=22).pack(side='left',padx=4);ttk.Button(top,text='Применить режим',command=lambda:self.send('MODE '+MODES[self.mode.get()])).pack(side='left');self.low_power=tk.BooleanVar(value=False);ttk.Checkbutton(top,text='Низкая мощность (стол)',variable=self.low_power,command=lambda:self.send('POWER LOW' if self.low_power.get() else 'POWER MAX')).pack(side='left',padx=(12,0))
  self.dist=tk.StringVar(value='');ttk.Label(top,text='Расстояние, м').pack(side='left',padx=(18,2));ttk.Entry(top,textvariable=self.dist,width=8).pack(side='left');ttk.Button(top,text='Авто: 30 с',command=lambda:self.send('AUTO 30')).pack(side='left',padx=4)
  playback=ttk.Frame(self,padding=(8,0));playback.pack(fill='x');ttk.Button(playback,text='ОТКРЫТЬ ЛОГ',command=self.open_playback).pack(side='left');self.play_button=ttk.Button(playback,text='▶ ВОСПРОИЗВЕСТИ',command=self.toggle_playback,state='disabled');self.play_button.pack(side='left',padx=4);ttk.Button(playback,text='ЖИВОЙ ПРИЁМ',command=self.leave_playback).pack(side='left');ttk.Label(playback,textvariable=self.play_name,width=34).pack(side='left',padx=(12,4));ttk.Combobox(playback,textvariable=self.play_speed,values=['0.25×','0.5×','1×','2×','4×','8×'],state='readonly',width=6).pack(side='left',padx=4);self.play_slider=ttk.Scale(playback,style='Horizontal.TScale',from_=0,to=1,variable=self.play_pos,command=self.seek_playback);self.play_slider.pack(side='left',fill='x',expand=True,padx=(8,5));ttk.Label(playback,textvariable=self.play_time_text,width=13).pack(side='right')
  tabs=ttk.Notebook(self);tabs.pack(fill='both',expand=True,padx=8,pady=(4,8));flight=ttk.Frame(tabs);setup_tab=ttk.Frame(tabs);tabs.add(flight,text='  ПОЛЁТ  ');tabs.add(setup_tab,text='  ПОДГОТОВКА / КАЛИБРОВКА  ')
  bar=ttk.Frame(flight,padding=(0,4));bar.pack(fill='x');self.rec=tk.StringVar(value='ОСТАНОВЛЕНО');self.link=tk.StringVar(value='НЕТ ПАКЕТОВ');self.file=tk.StringVar(value='');
  ttk.Button(bar,text='НАЧАТЬ ЗАПИСЬ',command=self.start).pack(side='left');ttk.Button(bar,text='ОСТАНОВИТЬ ЗАПИСЬ',command=self.stop).pack(side='left',padx=5);ttk.Button(bar,text='ПЕРЕЗАГРУЗИТЬ БОРТ',command=self.reboot_mobile).pack(side='left',padx=(14,5));ttk.Label(bar,textvariable=self.rec).pack(side='left',padx=15);ttk.Label(bar,textvariable=self.link).pack(side='left')
  self.v={x:tk.StringVar(value='—') for x in ('rssi','loss','kbps','pps','mode','power','cpu','gap','duration')};self.s={x:tk.StringVar(value='—') for x in ('acc','gyro','raw_acc','raw_gyro','rpy','altitude','temp','humidity','battery','state','speed','vertical','net_accel','ins')};status_row=ttk.Frame(flight,padding=(0,0));status_row.pack(fill='x');grid=ttk.LabelFrame(status_row,text='Радиоканал',padding=10);grid.pack(fill='both',expand=True)
  for i,(t,k) in enumerate([('RSSI, dBm','rssi'),('Потери, %','loss'),('Поток, кбит/с','kbps'),('Пакеты/с','pps'),('Режим TX','mode'),('Мощность TX','power'),('CPU цикла','cpu'),('Макс. пауза, мс','gap'),('Запись','duration')]):grid.columnconfigure(i,weight=1);ttk.Label(grid,text=t).grid(row=0,column=i,sticky='w',padx=6);ttk.Label(grid,textvariable=self.v[k],font=('Segoe UI',14,'bold')).grid(row=1,column=i,sticky='w',padx=6)
  sensors=ttk.LabelFrame(flight,text='Телеметрия мобильной платы',padding=8);sensors.pack(fill='x',pady=(4,0));telemetry_values=ttk.Frame(sensors);telemetry_values.pack(side='left',fill='x',expand=True)
  for i,(t,k) in enumerate([('Высота ISA, м','altitude'),('Температура, °C','temp'),('Влажность, %','humidity'),('Батарея, V','battery'),('Roll / Pitch / Yaw, °','rpy')]):ttk.Label(telemetry_values,text=t).grid(row=0,column=i,sticky='w',padx=6);ttk.Label(telemetry_values,textvariable=self.s[k],font=('Segoe UI',10,'bold')).grid(row=1,column=i,sticky='w',padx=6)
  health=ttk.Frame(sensors);health.pack(side='right',padx=(12,0));ttk.Label(health,text='ДАТЧИКИ').pack(side='left',padx=(0,5));self.sensor_lamps={}
  for key in ('imu','bmp','sht','ins'):
   lamp=tk.Label(health,text='—',bg='#24180f',fg='#f1e6d5',relief='solid',bd=1,padx=6,pady=2,font=('Segoe UI',9,'bold'));lamp.pack(side='left',padx=2);self.sensor_lamps[key]=lamp
  inertial=ttk.LabelFrame(flight,text='Скорость по IMU / барометру — оценка, не навигационное измерение',padding=6);inertial.pack(fill='x',pady=(4,0))
  for i,(t,k) in enumerate([('Скорость 3D','speed'),('Вертикальная, ↑','vertical'),('Ускорение без g','net_accel'),('Статус оценки','ins')]):ttk.Label(inertial,text=t).grid(row=0,column=i,sticky='w',padx=6);ttk.Label(inertial,textvariable=self.s[k],font=('Segoe UI',10,'bold')).grid(row=1,column=i,sticky='w',padx=6)
  view=ttk.Frame(flight,padding=(0,8));view.pack(fill='both',expand=True);charts=ttk.Frame(view,width=440);charts.pack(side='left',fill='y',padx=(0,8));charts.pack_propagate(False);charts.columnconfigure(0,weight=1);charts.columnconfigure(1,weight=1);self.alt_canvas=tk.Canvas(charts,width=210,height=104,bg='#130e0a',highlightthickness=0);self.alt_canvas.grid(row=0,column=0,sticky='ew',padx=(0,3),pady=(0,4));self.speed_canvas=tk.Canvas(charts,width=210,height=104,bg='#130e0a',highlightthickness=0);self.speed_canvas.grid(row=0,column=1,sticky='ew',padx=(3,0),pady=(0,4));self.g_canvas=tk.Canvas(charts,width=210,height=104,bg='#130e0a',highlightthickness=0);self.g_canvas.grid(row=1,column=0,sticky='ew',padx=(0,3),pady=4);self.pps_canvas=tk.Canvas(charts,width=210,height=104,bg='#130e0a',highlightthickness=0);self.pps_canvas.grid(row=1,column=1,sticky='ew',padx=(3,0),pady=4);self.rssi_canvas=tk.Canvas(charts,width=440,height=104,bg='#130e0a',highlightthickness=0);self.rssi_canvas.grid(row=2,column=0,columnspan=2,sticky='ew',pady=(4,0));self.imu_canvas=tk.Canvas(view,width=660,height=470,bg='#130e0a',highlightthickness=0,cursor='fleur');self.imu_canvas.pack(side='right',fill='both',expand=True);self.imu_canvas.bind('<ButtonPress-1>',lambda e:self.scene_begin(e,'orbit'));self.imu_canvas.bind('<B1-Motion>',self.scene_drag_move);self.imu_canvas.bind('<ButtonPress-3>',lambda e:self.scene_begin(e,'pan'));self.imu_canvas.bind('<B3-Motion>',self.scene_drag_move);self.imu_canvas.bind('<MouseWheel>',self.scene_wheel);self.imu_canvas.bind('<Button-4>',lambda e:self.scene_wheel(e,1));self.imu_canvas.bind('<Button-5>',lambda e:self.scene_wheel(e,-1))
  ttk.Label(setup_tab,text='Перед испытанием: проверьте датчики, задайте массу и опорную высоту, затем при необходимости выполните калибровку IMU.',wraplength=900,justify='left').pack(anchor='w',padx=12,pady=(12,6))
  calibration=ttk.LabelFrame(setup_tab,text='Калибровка IMU: 6 положений',padding=10);calibration.pack(fill='x',padx=12,pady=4);ttk.Label(calibration,textvariable=self.cal_text,wraplength=650,justify='left').pack(side='left',fill='x',expand=True);self.cal_button=ttk.Button(calibration,text='НАЧАТЬ КАЛИБРОВКУ',command=self.cal_action);self.cal_button.pack(side='right',padx=4)
  baro=ttk.LabelFrame(setup_tab,text='Калибровка барометра',padding=10);baro.pack(fill='x',padx=12,pady=4);ttk.Label(baro,textvariable=self.baro_pressure,font=('Segoe UI',10,'bold')).pack(side='left',padx=(0,16));ttk.Label(baro,text='Опорная высота, м').pack(side='left');ttk.Entry(baro,textvariable=self.baro_ref,width=10).pack(side='left',padx=6);ttk.Button(baro,text='КАЛИБРОВАТЬ ВЫСОТУ',command=self.calibrate_baro).pack(side='left');ttk.Label(baro,textvariable=self.baro_status).pack(side='left',padx=12)
  configuration=ttk.LabelFrame(setup_tab,text='Параметры мобильной установки',padding=10);configuration.pack(fill='x',padx=12,pady=4);ttk.Label(configuration,text='Масса, кг').pack(side='left');ttk.Entry(configuration,textvariable=self.mass,width=10).pack(side='left',padx=6);ttk.Label(configuration,text='Сохраняется вместе с записью и итоговым отчётом.').pack(side='left',padx=8)
  imu_diag=ttk.LabelFrame(setup_tab,text='Диагностика IMU: исходные оси датчика и тело FRD',padding=8);imu_diag.pack(fill='x',padx=12,pady=4)
  for i,(t,k) in enumerate([('ACC raw, м/с²','raw_acc'),('Gyro raw, рад/с','raw_gyro'),('Gyro FRD, рад/с','gyro'),('ACC FRD, м/с²','acc')]):ttk.Label(imu_diag,text=t).grid(row=0,column=i,sticky='w',padx=8);ttk.Label(imu_diag,textvariable=self.s[k],font=('Segoe UI',10,'bold')).grid(row=1,column=i,sticky='w',padx=8)
  self.after(50,self.drain);self.after(40,self.render_imu);self.after(300,self.auto_connect)
 def candidates(self):
  ps=list(list_ports.comports());named=[p.device for p in ps if p.device=='COM9'];esp=[p.device for p in ps if p.vid==0x303A and p.device not in named];return named+esp+[p.device for p in ps if p.device not in named+esp]
 def load_settings(self):
  try:
   saved=json.loads(SETTINGS_PATH.read_text(encoding='utf-8'));qnh=float(saved.get('qnh_pa',QNH_PA));ref=float(saved.get('baro_reference_altitude_m',175));mass=str(saved.get('mass_kg',''))
   if 80000<qnh<110000:self.qnh_pa=qnh
   if -500<ref<10000:self.baro_ref.set(f'{ref:g}')
   self.mass.set(mass);self.baro_calibrated=bool(saved.get('baro_calibrated',False));self.baro_status.set(f'QNH {self.qnh_pa:.1f} Pa')
  except (OSError,ValueError,TypeError,json.JSONDecodeError):self.baro_status.set(f'QNH {self.qnh_pa:.1f} Pa')
 def save_settings(self):
  try:SETTINGS_PATH.write_text(json.dumps({'qnh_pa':self.qnh_pa,'baro_reference_altitude_m':self.baro_ref.get(),'baro_calibrated':self.baro_calibrated,'mass_kg':self.mass.get()},ensure_ascii=False,indent=2),encoding='utf-8')
  except OSError:pass
 def calibrate_baro(self):
  try:altitude=float(self.baro_ref.get().replace(',','.'))
  except ValueError:messagebox.showerror('Барометр','Введите опорную высоту числом в метрах.');return
  if not self.current_pressure or not -500<altitude<10000:
   messagebox.showerror('Барометр','Нужны текущие данные BMP280 и высота от −500 до 10 000 м.');return
  ratio=1-altitude/44330.
  if ratio<=0:messagebox.showerror('Барометр','Некорректная высота.');return
  self.qnh_pa=self.current_pressure/(ratio**(1/.190294957));self.baro_ref.set(f'{altitude:g}');self.baro_calibrated=True;self.baro_status.set(f'Сохранено: QNH {self.qnh_pa:.1f} Pa');self.save_settings();self.draw()
 def set_sensor_lamp(self,key,text,color):
  lamp=self.sensor_lamps.get(key)
  if lamp:lamp.configure(text=text,fg=color)
 def update_sensor_health(self,flags,est,ins):
  white='#f1e6d5';green='#69d49a';red='#ef6b62';self.set_sensor_lamp('imu','IMU · CAL' if flags&1 and est&4 else ('IMU · ЕСТЬ' if flags&1 else 'IMU · НЕТ'),green if flags&1 and est&4 else (white if flags&1 else red));self.set_sensor_lamp('bmp','BARO · CAL' if flags&2 and self.baro_calibrated else ('BARO · ЕСТЬ' if flags&2 else 'BARO · НЕТ'),green if flags&2 and self.baro_calibrated else (white if flags&2 else red));self.set_sensor_lamp('sht','T/H · ЕСТЬ' if flags&4 else 'T/H · НЕТ',white if flags&4 else red);self.set_sensor_lamp('ins','ОЦ · ГОТОВА' if ins&2 else ('ОЦ · СТАРТ' if flags&1 else 'ОЦ · НЕТ'),green if ins&2 else (white if flags&1 else red))
 def load_vehicle_mesh(self):
  path=APP_DIR/'vehicle.stl'
  try:
   data=path.read_bytes();count=struct.unpack_from('<I',data,80)[0]
   if len(data)<84+count*50 or count==0:return []
   low=[float('inf')]*3;high=[float('-inf')]*3
   for i in range(count):
    vals=struct.unpack_from('<9f',data,96+i*50)
    for j in range(0,9,3):
     for axis in range(3):low[axis]=min(low[axis],vals[j+axis]);high[axis]=max(high[axis],vals[j+axis])
   # Build a coherent low-poly mesh, rather than sampling isolated original
   # triangles.  The longest STL axis becomes vehicle +X (the nose).
   span=[max(high[a]-low[a],1e-6) for a in range(3)];center=[(low[i]+high[i])*.5 for i in range(3)]
   long_axis=max(range(3),key=lambda a:span[a]);side_axes=[a for a in range(3) if a!=long_axis];grid=[4,4,4];grid[long_axis]=22;grid=tuple(grid);cross_scale=min(6.0,max(1.0,.8*span[long_axis]/max(span[side_axes[0]],span[side_axes[1]])));vertices={};faces={};end_radius=[[],[]]
   for i in range(count):
    vals=struct.unpack_from('<9f',data,96+i*50);keys=[]
    for j in range(0,9,3):
     p=(vals[j],vals[j+1],vals[j+2]);u=(p[long_axis]-low[long_axis])/span[long_axis];radius=(p[side_axes[0]]-center[side_axes[0]])**2+(p[side_axes[1]]-center[side_axes[1]])**2
     if u<.12:end_radius[0].append(radius)
     elif u>.88:end_radius[1].append(radius)
     key=tuple(min(grid[a]-1,int((p[a]-low[a])*grid[a]/span[a])) for a in range(3));s=vertices.setdefault(key,[0.,0.,0.,0.]);s[0]+=p[0];s[1]+=p[1];s[2]+=p[2];s[3]+=1;keys.append(key)
    if len(set(keys))==3:faces.setdefault(tuple(sorted(keys)),tuple(keys))
   # The tapered end is the rocket nose.  This removes the arbitrary STL
   # winding/sign from the visual body coordinate system.
   nose_sign=1 if end_radius[1] and end_radius[0] and sum(end_radius[1])/len(end_radius[1])<sum(end_radius[0])/len(end_radius[0]) else -1;mesh=[]
   for tri in faces.values():
    out=[]
    for key in tri:
     s=vertices[key];p=(s[0]/s[3],s[1]/s[3],s[2]/s[3]);out.append((nose_sign*1.72*(p[long_axis]-center[long_axis])/span[long_axis],cross_scale*(p[side_axes[0]]-center[side_axes[0]])/span[long_axis],cross_scale*(p[side_axes[1]]-center[side_axes[1]])/span[long_axis]))
    mesh.append(out)
   return mesh
  except (OSError,ValueError,struct.error):return []
 def open_port(self,name,silent=True):
  if self.ser and self.ser.is_open:return True
  try:
   port=serial.Serial();port.port=name;port.baudrate=921600;port.timeout=.15;port.open();port.dtr=True;port.rts=True;port.reset_input_buffer();self.ser=port;self.port.set(name);threading.Thread(target=self.reader,daemon=True).start();self.link.set('ПОДКЛЮЧЕНО — ОЖИДАНИЕ ПАКЕТОВ');return True
  except Exception as e:
   self.ser=None
   if not silent:messagebox.showerror('COM',str(e))
   return False
 def connect(self):self.open_port(self.port.get(),False)
 def auto_connect(self):
  if not(self.ser and self.ser.is_open):
   for name in self.candidates():
    if self.open_port(name,True):break
  self.after(1500,self.auto_connect)
 def reader(self):
  while self.ser:
   try:
    x=self.ser.readline().decode(errors='replace').strip()
    if x:self.q.put(x)
   except:
    self.ser=None;self.q.put('PORT,status=disconnected');break
 def send(self,x):
  if self.playback_active:
   self.link.set('ВОСПРОИЗВЕДЕНИЕ — радиокоманда не отправлена');return False
  if not(self.ser and self.ser.is_open):
   self.link.set('БАЗА НЕ ПОДКЛЮЧЕНА');return False
  try:self.ser.write((x+'\n').encode());self.ser.flush();return True
  except:
   self.ser=None;self.link.set('ОШИБКА ПОРТА — автоповтор');return False
 def reboot_mobile(self):
  if self.send('REBOOT'):self.link.set('КОМАНДА ПЕРЕЗАГРУЗКИ ОТПРАВЛЕНА — жду борт…')
 def pose_text(self,step):
  title,text=CAL_POSES[min(step,5)];return title+' — '+text+' Не двигайте плату и нажмите «ДАЛЕЕ».'
 def cal_action(self):
  if not self.cal_running:
   if not self.send('CAL START'):self.cal_text.set('База не подключена: подождите автоподключение и повторите.');return
   self.cal_running=True;self.cal_seen_active=False;self.cal_step=0;self.cal_phase='waiting_start';self.cal_start_attempts=1;self.cal_start_at=time.monotonic();self.cal_button.configure(text='ОТКРЫВАЮ…',state='disabled');self.cal_text.set('Открываю калибровку на мобильной плате…');return
  if self.cal_phase=='waiting_start':return
  if self.cal_phase=='capturing':return
  if not self.send('CAL CAPTURE'):
   self.cal_text.set('База не подключена: подождите автоподключение и повторите «ДАЛЕЕ».');return
  self.cal_phase='capturing';self.cal_button.configure(state='disabled');self.cal_text.set(f'Запоминаю положение {self.cal_step+1} из 6 — держите плату неподвижно 1 секунду…')
 def service_calibration_start(self):
  if not(self.cal_running and self.cal_phase=='waiting_start') or time.monotonic()-self.cal_start_at<.7:return
  if self.cal_start_attempts>=5:
   self.cal_running=False;self.cal_seen_active=False;self.cal_phase='idle';self.cal_button.configure(text='НАЧАТЬ КАЛИБРОВКУ',state='normal');self.cal_text.set('Мобильная плата не подтвердила запуск. Проверьте, что идёт приём телеметрии, и повторите.');return
  if self.send('CAL START'):
   self.cal_start_attempts+=1;self.cal_start_at=time.monotonic();self.cal_text.set(f'Открываю калибровку на мобильной плате… попытка {self.cal_start_attempts}/5.')
 def update_calibration(self,flags):
  active=bool(flags&8);done=(flags>>4)&7;collecting=bool(flags&128);quality=(flags>>8)&255
  if active:
   self.cal_seen_active=True;self.cal_running=True;self.cal_step=min(done,6)
   if collecting:
    self.cal_phase='capturing';self.cal_button.configure(state='disabled');self.cal_text.set(f'Запоминаю положение {min(done+1,6)} из 6 — держите плату неподвижно 1 секунду…')
   elif done<6:
    self.cal_phase='ready';self.cal_button.configure(text='ДАЛЕЕ',state='normal');self.cal_text.set(self.pose_text(done))
  elif self.cal_running and self.cal_seen_active:
   self.cal_running=False;self.cal_seen_active=False;self.cal_phase='idle';self.cal_button.configure(text='НАЧАТЬ КАЛИБРОВКУ',state='normal')
   self.cal_text.set(f'Готово. Оси «нос Type-C / право / верх» сохранены в мобильной плате: качество {quality}/100.' if quality>=70 else 'Калибровка не сохранена: одно из шести положений не стало противоположным ожидаемому либо плата двигалась. Повторите, строго следуя направлению Type-C.')
  elif not self.cal_running:
   self.cal_text.set(f'IMU ОК: калибровка сохранена в мобильной плате, качество {quality}/100.' if quality>=70 and flags&1 else ('IMU найдена, но не откалибрована. Выполните 6 положений.' if flags&1 else 'IMU не найдена: калибровка недоступна.'))
 def start(self):
  if self.playback_active:
   messagebox.showinfo('Запись', 'Сначала нажмите «ЖИВОЙ ПРИЁМ»: во время воспроизведения лог не записывается.');return
  if self.record:return
  log_dir=APP_DIR/'logs';log_dir.mkdir(exist_ok=True);mode=MODES[self.mode.get()];kb={'NORMAL':'28','RESERVE':'40','STRESS':'60'}[mode]
  while True:
   self.run+=1;name=f"ROKI_{mode}_{kb}kbps_{datetime.now():%Y-%m-%d_%H%M%S}_{self.run:03}.csv"
   try:self.record=open(log_dir/name,'x',newline='',encoding='utf-8');break
   except FileExistsError:continue
  self.writer=csv.writer(self.record);self.writer.writerow(['received_at','tx_ms','distance_m','sequence','rssi_dbm','length','crc','mode','gap_ms','loss_pct','kbps','sensor_flags','estimator_status','acc_sensor_x_ms2','acc_sensor_y_ms2','acc_sensor_z_ms2','gyro_sensor_x_rads','gyro_sensor_y_rads','gyro_sensor_z_rads','acc_body_x_ms2','acc_body_y_ms2','acc_body_z_ms2','gyro_body_x_rads','gyro_body_y_rads','gyro_body_z_rads','roll_deg','pitch_deg','yaw_deg','pressure_pa','altitude_isa_m','temperature_c','humidity_pct','battery_voltage_v','speed_imu_est_mps','speed_imu_est_kmh','vertical_speed_mps','acceleration_mps2','inertial_status','vertical_speed_quality','receiver_queue_overflow','usb_drops','protocol_errors','mass_kg','baro_qnh_pa','baro_reference_altitude_m']);self.record_path=log_dir/name;self.record_stats={'rx':0,'rssi_sum':0.,'rssi_min':0,'max_gap':0,'mode':''};self.reset_trajectory();self.imu_rendered=None;self.draw_imu();self.rec.set('ЗАПИСЬ ИДЁТ');self.file.set(name);self.started=time.monotonic();self.save_settings()
 def stop(self):
  if self.record:
   duration=time.monotonic()-self.started;path=self.record_path;stats=self.record_stats;self.record.flush();self.record.close();self.record=None;self.writer=None
   actual=stats.get('mode') or 'UNKNOWN'
   if path and actual in ('NORMAL','RESERVE','STRESS'):
    kb={'NORMAL':'28','RESERVE':'40','STRESS':'60'}[actual];parts=path.stem.split('_');renamed=path.with_name(f"ROKI_{actual}_{kb}kbps_{parts[3]}_{parts[4]}_{parts[5]}.csv")
    try:path.rename(renamed);path=renamed;self.record_path=renamed;self.file.set(renamed.name)
    except OSError:pass
   if path:
    average=stats['rssi_sum']/stats['rx'] if stats['rx'] else 0
    try:path.with_suffix('.summary.txt').write_text(f'ROKI summary\nfile: {path.name}\nduration_s: {duration:.1f}\nreceived_packets: {stats["rx"]}\nloss_pct: {self.v["loss"].get()}\nrssi_avg_dbm: {average:.2f}\nrssi_min_dbm: {stats["rssi_min"]}\nmax_gap_ms: {stats["max_gap"]}\ndistance_m: {self.dist.get()}\nmass_kg: {self.mass.get()}\nbaro_qnh_pa: {self.qnh_pa:.2f}\nbaro_reference_altitude_m: {self.baro_ref.get()}\n',encoding='utf-8')
    except OSError:pass
  self.rec.set('ОСТАНОВЛЕНО')
 def open_playback(self):
  if self.record:
   messagebox.showinfo('Воспроизведение', 'Сначала остановите текущую запись, чтобы не смешать живые данные с логом.');return
  path=filedialog.askopenfilename(title='Выберите журнал ROKI',initialdir=APP_DIR/'logs',filetypes=[('Журналы ROKI','ROKI_*.csv'),('CSV','*.csv'),('Все файлы','*.*')])
  if not path:return
  try:
   with Path(path).open('r',newline='',encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
   rows=[r for r in rows if r.get('received_at')]
   if not rows:raise ValueError('В файле нет строк телеметрии.')
   origin=datetime.fromisoformat(rows[0]['received_at'])
   times=[];last=0.
   for i,row in enumerate(rows):
    try:t=max(last,(datetime.fromisoformat(row['received_at'])-origin).total_seconds())
    except (ValueError,TypeError):t=last+.02 if i else 0.
    times.append(t);last=t
  except (OSError,ValueError,csv.Error) as e:
   messagebox.showerror('Воспроизведение',f'Не удалось прочитать CSV:\n{e}');return
  self.play_rows=rows;self.play_times=times;self.play_index=-1;self.play_time=0.;self.playing=False;self.playback_active=True;self.play_last_second=-1;self.play_clock=time.monotonic();self.play_name.set(Path(path).name);self.play_slider.configure(to=max(times[-1],.01));self.play_button.configure(state='normal',text='▶ ВОСПРОИЗВЕСТИ');self._set_play_position(0.);self._rebuild_playback(0);self.link.set('ЛОГ ЗАГРУЖЕН — ПАУЗА');self.draw()
 def playback_speed(self):
  try:return float(self.play_speed.get().replace('×',''))
  except ValueError:return 1.
 def format_play_time(self,value):
  value=max(0,int(value));return f'{value//60:02}:{value%60:02}'
 def _set_play_position(self,value):
  self.play_seeking=True;self.play_pos.set(value);self.play_seeking=False;total=self.play_times[-1] if self.play_times else 0.;self.play_time_text.set(f'{self.format_play_time(value)} / {self.format_play_time(total)}')
 def toggle_playback(self):
  if not self.playback_active or not self.play_rows:return
  if self.play_time>=self.play_times[-1]:self._rebuild_playback(0);self._set_play_position(0.)
  self.playing=not self.playing;self.play_clock=time.monotonic();self.play_button.configure(text='❚❚ ПАУЗА' if self.playing else '▶ ВОСПРОИЗВЕСТИ');self.link.set('ВОСПРОИЗВЕДЕНИЕ' if self.playing else 'ЛОГ — ПАУЗА')
 def seek_playback(self,value):
  if self.play_seeking or not self.playback_active or not self.play_rows:return
  try:target=float(value)
  except (TypeError,ValueError):return
  self.playing=False;self.play_button.configure(text='▶ ВОСПРОИЗВЕСТИ');self._rebuild_playback(target);self._set_play_position(target);self.link.set('ЛОГ — ПАУЗА');self.draw()
 def _reset_visual_history(self):
  self.rssi.clear();self.altitude_history.clear();self.g_history.clear();self.speed_history.clear();self.pps_history.clear();self.roll=self.pitch=self.yaw=0.;self.reset_trajectory();self.imu_rendered=None
 def reset_trajectory(self):
  self.trajectory_generation+=1;self.trajectory=[(0.,0.,0.)];self.trajectory_position=[0.,0.,0.];self.trajectory_velocity=[0.,0.,0.];self.trajectory_accel=[0.,0.,0.];self.trajectory_at=time.monotonic();self.trajectory_tx_ms=None
 def integrate_trajectory(self,ax,ay,az,roll,pitch,yaw,vertical,ins,dt):
  if dt<=0 or dt>.25:return
  # Body FRD -> world NED, then world NEU.  Acceleration includes gravity.
  cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
  north=cp*cy*ax+(sr*sp*cy-cr*sy)*ay+(cr*sp*cy+sr*sy)*az
  east=cp*sy*ax+(sr*sp*sy+cr*cy)*ay+(cr*sp*sy-sr*cy)*az
  down=-sp*ax+sr*cp*ay+cr*cp*az+9.80665
  acc=[north,east,-down]
  if ins&8:self.trajectory_velocity=[0.,0.,0.];self.trajectory_accel=[0.,0.,0.]
  elif ins&2 and not(ins&1024):
   for i in (0,1):
    if abs(acc[i])<.22:acc[i]=0.
    self.trajectory_position[i]+=self.trajectory_velocity[i]*dt+.5*(self.trajectory_accel[i]+acc[i])*.5*dt*dt
    self.trajectory_velocity[i]+=(self.trajectory_accel[i]+acc[i])*.5*dt
   self.trajectory_accel=acc
  if ins&2:self.trajectory_position[2]+=vertical*dt
  if not self.trajectory or math.dist(self.trajectory[-1],self.trajectory_position)>.015:self.trajectory.append(tuple(self.trajectory_position));self.trajectory=self.trajectory[-1800:]
 def _rebuild_playback(self,target):
  if not self.play_rows:return
  target=max(0.,min(target,self.play_times[-1]));index=bisect.bisect_right(self.play_times,target)-1;self._reset_visual_history();self.play_last_second=-1
  first=max(0,index-3000)
  for i in range(first,index+1):self.apply_rx(self.playback_packet(i),from_log=True,update_rate=False,trajectory_dt=(self.play_times[i]-self.play_times[i-1] if i>first else 0.))
  self.play_index=index;self.play_time=target
  if index>=0:self._update_play_rates(index)
 def update_playback(self):
  if not self.playback_active or not self.playing or not self.play_rows:return False
  target=min(self.play_times[-1],self.play_time+(time.monotonic()-self.play_clock)*self.playback_speed());changed=False
  while self.play_index+1<len(self.play_rows) and self.play_times[self.play_index+1]<=target:
   self.play_index+=1;self.apply_rx(self.playback_packet(self.play_index),from_log=True,update_rate=False,trajectory_dt=(self.play_times[self.play_index]-self.play_times[self.play_index-1] if self.play_index else 0.));changed=True
  if changed:self._update_play_rates(self.play_index)
  self.play_time=target;self.play_clock=time.monotonic();self._set_play_position(target)
  if target>=self.play_times[-1]:self.playing=False;self.play_button.configure(text='▶ ВОСПРОИЗВЕСТИ');self.link.set('ЛОГ — КОНЕЦ')
  return changed
 def playback_packet(self,index):
  row=self.play_rows[index]
  def value(name,default='0'):return str(row.get(name,default) or default)
  def scaled(name,factor):return str(round(self.number(row.get(name))*factor))
  # CSV stores SI m/s², independent of the binary protocol's compact scale.
  acc_scale=100
  return {'seq':value('sequence'),'tx_ms':value('tx_ms'),'rssi':value('rssi_dbm','-127'),'len':value('length'),'crc':value('crc','OK'),'mode':value('mode'),'gap':value('gap_ms'),'loss':value('loss_pct'),'flags':value('sensor_flags'),'est':value('estimator_status'),'rawax':scaled('acc_sensor_x_ms2',acc_scale),'raway':scaled('acc_sensor_y_ms2',acc_scale),'rawaz':scaled('acc_sensor_z_ms2',acc_scale),'rawgx':scaled('gyro_sensor_x_rads',1000),'rawgy':scaled('gyro_sensor_y_rads',1000),'rawgz':scaled('gyro_sensor_z_rads',1000),'ax':scaled('acc_body_x_ms2',acc_scale),'ay':scaled('acc_body_y_ms2',acc_scale),'az':scaled('acc_body_z_ms2',acc_scale),'gx':scaled('gyro_body_x_rads',1000),'gy':scaled('gyro_body_y_rads',1000),'gz':scaled('gyro_body_z_rads',1000),'roll':scaled('roll_deg',100),'pitch':scaled('pitch_deg',100),'yaw':scaled('yaw_deg',100),'pressure':value('pressure_pa'),'temp_centi':str(int(self.number(row.get('temperature_c'))*100)),'humidity_centi':str(int(self.number(row.get('humidity_pct'))*100)),'speed':scaled('speed_imu_est_mps',100),'battery_mv':scaled('battery_voltage_v',1000),'vertical':scaled('vertical_speed_mps',100),'net_accel':scaled('acceleration_mps2',100),'ins_status':value('inertial_status')}
 def number(self,value,default=0.):
  try:return float(value)
  except (TypeError,ValueError):return default
 def _update_play_rates(self,index):
  now=self.play_times[index];first=index
  while first>0 and self.play_times[first-1]>=now-1.:first-=1
  span=max(now-self.play_times[first],.02);rows=self.play_rows[first:index+1];pps=len(rows)/span;kbps=sum(self.number(r.get('length')) for r in rows)*8/1000/span;self.v['pps'].set(f'{pps:.1f}');self.v['kbps'].set(f'{kbps:.2f}')
  second=int(now)
  if second!=self.play_last_second:self.pps_history.append(pps);self.play_last_second=second
  elif self.pps_history:self.pps_history[-1]=pps
 def leave_playback(self):
  if not self.playback_active:return
  self.playing=False;self.playback_active=False;self.play_button.configure(state='disabled',text='▶ ВОСПРОИЗВЕСТИ');self.play_name.set('Лог не загружен');self.play_time_text.set('00:00 / 00:00');self.play_slider.configure(to=1);self._set_play_position(0.);self.last=time.monotonic();self.link.set('ПОДКЛЮЧЕНО — ОЖИДАНИЕ ПАКЕТОВ' if self.ser and self.ser.is_open else 'НЕТ ПАКЕТОВ')
 def drain(self):
  batch=[]
  for _ in range(120):
   try:batch.append(self.q.get_nowait())
   except queue.Empty:break
  for line in batch:self.handle(line)
  self.service_calibration_start()
  replay_changed=self.update_playback()
  if (batch or replay_changed) and time.monotonic()>=self.next_render:self.draw();self.next_render=time.monotonic()+.25
  if self.record:
   self.v['duration'].set(f'{time.monotonic()-self.started:.0f} s')
   if time.monotonic()-self.last_flush>=1:self.record.flush();self.last_flush=time.monotonic()
  if not self.playback_active and time.monotonic()-self.last>1:self.link.set('НЕТ ПАКЕТОВ')
  self.after(50,self.drain)
 def handle(self,line):
  if self.playback_active and line.startswith(('RX','STATUS')):return
  d=dict(x.split('=',1) for x in line.split(',')[1:] if '=' in x)
  if line.startswith('RX') and 's' in d:
    a=d.get('a','0/0/0').split('/');w=d.get('w','0/0/0').split('/');ra=d.get('ra','0/0/0').split('/');rw=d.get('rw','0/0/0').split('/');rpy=d.get('rpy','0/0/0').split('/');ins=d.get('i','0/0/0/0/0').split('/');d.update({'seq':d.get('s','0'),'tx_ms':d.get('x',''),'rssi':d.get('r','-127'),'len':d.get('n','0'),'mode':d.get('m','0'),'gap':d.get('g','0'),'loss':d.get('o','0'),'flags':d.get('f','0'),'ax':a[0],'ay':a[1],'az':a[2],'gx':w[0],'gy':w[1],'gz':w[2],'rawax':ra[0],'raway':ra[1],'rawaz':ra[2],'rawgx':rw[0],'rawgy':rw[1],'rawgz':rw[2],'roll':rpy[0],'pitch':rpy[1],'yaw':rpy[2],'est':d.get('e','0'),'pressure':d.get('p','0'),'temp_centi':d.get('t','0'),'humidity_centi':d.get('h','0'),'speed':ins[0],'battery_mv':ins[1],'vertical':ins[2],'net_accel':ins[3],'ins_status':ins[4],'crc':'OK'})
  if line.startswith('RX'):
    self.apply_rx(d)
  elif line.startswith('STATUS'):
   self.v['pps'].set(d.get('pps','—'));self.v['kbps'].set(d.get('kbps','—'));self.v['gap'].set(d.get('max_gap_ms',self.v['gap'].get()));self.rx_queue_overflow=int(d.get('queue_overflow','0'));self.usb_drops=int(d.get('usb_drops','0'));self.protocol_errors=int(d.get('protocol_err','0'))
   try:self.pps_history.append(float(d.get('pps','0')))
   except ValueError:pass
 def apply_rx(self,d,from_log=False,update_rate=True,trajectory_dt=None):
  if not from_log:self.last=time.monotonic();self.link.set('ПРИЁМ')
  mode_name={'0':'NORMAL','1':'RESERVE','2':'STRESS'}.get(str(d.get('mode','')),str(d.get('mode','—')))
  if self.record and mode_name in ('NORMAL','RESERVE','STRESS'):self.record_stats['mode']=mode_name
  self.v['rssi'].set(d.get('rssi','—'));self.v['loss'].set(d.get('loss','—'));self.v['mode'].set(mode_name);self.v['gap'].set(d.get('gap','—'));self.rssi.append(int(d.get('rssi','-127')))
  ax,ay,az=[int(d.get(k,0))/100 for k in ('ax','ay','az')];gx,gy,gz=[int(d.get(k,0))/1000 for k in ('gx','gy','gz')]
  rawax,raway,rawaz=[int(d.get(k,0))/100 for k in ('rawax','raway','rawaz')];rawgx,rawgy,rawgz=[int(d.get(k,0))/1000 for k in ('rawgx','rawgy','rawgz')]
  roll,pitch,yaw=[int(d.get(k,0))/100 for k in ('roll','pitch','yaw')];yaw_signed=(yaw+180)%360-180;pressure=int(d.get('pressure',0));temp=int(d.get('temp_centi',0))/100;humidity=int(d.get('humidity_centi',0))/100;battery=int(d.get('battery_mv',0))/1000;speed_raw=int(d.get('speed',65535));speed=None if speed_raw==65535 else speed_raw/100;speed_kmh=None if speed is None else speed*3.6;vertical=int(d.get('vertical',0))/100;net_accel=int(d.get('net_accel',0))/100;flags=int(d.get('flags',0));est=int(d.get('est',0));ins=int(d.get('ins_status',0));self.low_power.set(bool(est&(1<<8)));self.v['power'].set('НИЗКАЯ · 2 dBm' if est&(1<<8) else 'МАКС. · 21 dBm');self.v['cpu'].set(f'{(est>>9)&0x7F}%')
  if not from_log:self.update_calibration(flags)
  quality=('INVALID','DEGRADED','VALID','INVALID')[(ins>>8)&3];self.current_pressure=pressure;self.baro_pressure.set(f'Давление {pressure:,} Pa'.replace(',',' ')) if pressure else self.baro_pressure.set('Давление — Pa');altitude=44330*(1-(pressure/self.qnh_pa)**0.190294957) if pressure>0 else 0;self.altitude_history.append(altitude);self.g_history.append(math.sqrt(ax*ax+ay*ay+az*az)/9.80665);self.speed_history.append(vertical)
  self.roll=math.radians(roll);self.pitch=math.radians(pitch);self.yaw=math.radians(yaw)
  if (self.record and not from_log) or from_log:
   if trajectory_dt is None:
    tx=self.number(d.get('tx_ms'),-1);dt=(tx-self.trajectory_tx_ms)/1000 if tx>=0 and self.trajectory_tx_ms is not None else 0.;self.trajectory_tx_ms=tx if tx>=0 else self.trajectory_tx_ms
    if dt<=0 or dt>.25:now=time.monotonic();dt=max(0.,min(now-self.trajectory_at,.25));self.trajectory_at=now
   else:dt=trajectory_dt
   self.integrate_trajectory(ax,ay,az,self.roll,self.pitch,self.yaw,vertical,ins,dt)
  ins_text=('ИНИЦ ' if ins&1 else '')+('ГОТОВ ' if ins&2 else '')+('БАРО ' if ins&4 else '')+('ZUPT ' if ins&8 else '')+('SAT! ' if ins&16 else '')+('Δt! ' if ins&32 else '')+('ЛИМИТ ' if ins&64 else '')+('BARO! ' if ins&128 else '')+('ROT ' if ins&1024 else '')+quality
  self.s['raw_acc'].set(f'{rawax:.3f} {raway:.3f} {rawaz:.3f}');self.s['raw_gyro'].set(f'{rawgx:.3f} {rawgy:.3f} {rawgz:.3f}');self.s['acc'].set(f'{ax:.3f} {ay:.3f} {az:.3f}');self.s['gyro'].set(f'{gx:.3f} {gy:.3f} {gz:.3f}');self.s['rpy'].set(f'{roll:+.1f}  {pitch:+.1f}  {yaw_signed:+.1f}');self.s['altitude'].set(f'{altitude:.1f}');self.s['temp'].set(f'{temp:.2f}');self.s['humidity'].set(f'{humidity:.2f}');self.s['battery'].set(f'{battery:.3f}' if battery else '—');self.s['state'].set(('IMU ' if flags&1 else 'IMU— ')+('BMP ' if flags&2 else 'BMP— ')+('SHT ' if flags&4 else 'SHT— ')+('ID OK ' if est&1 else 'ID? ')+('EST ' if est&2 else 'EST— ')+('CAL' if est&4 else 'CAL—'));self.update_sensor_health(flags,est,ins);self.s['speed'].set('недоступна');self.s['vertical'].set(f'{vertical:+.2f} м/с · {quality}');self.s['net_accel'].set(f'{net_accel:.2f} м/с²');self.s['ins'].set(ins_text.strip() or 'INVALID')
  if self.writer and not from_log:
   self.writer.writerow([datetime.now().isoformat(timespec='milliseconds'),d.get('tx_ms',''),self.dist.get(),d.get('seq'),d.get('rssi'),d.get('len'),d.get('crc'),d.get('mode'),d.get('gap'),d.get('loss'),self.v['kbps'].get(),flags,est,rawax,raway,rawaz,rawgx,rawgy,rawgz,ax,ay,az,gx,gy,gz,roll,pitch,yaw,pressure,altitude,temp,humidity,battery,'' if speed is None else speed,'' if speed_kmh is None else speed_kmh,vertical,net_accel,ins,quality,self.rx_queue_overflow,self.usb_drops,self.protocol_errors,self.mass.get(),self.qnh_pa,self.baro_ref.get()])
   self.record_stats['rx']+=1;self.record_stats['rssi_sum']+=int(d.get('rssi',-127));self.record_stats['rssi_min']=min(self.record_stats['rssi_min'],int(d.get('rssi',-127))) if self.record_stats['rx']>1 else int(d.get('rssi',-127));self.record_stats['max_gap']=max(self.record_stats['max_gap'],int(d.get('gap',0)))
 def render_imu(self):
  self.draw_imu();self.after(80 if self.vehicle_mesh else 40,self.render_imu)
 def scene_begin(self,event,mode):
  self.scene_drag=(mode,event.x,event.y)
 def scene_drag_move(self,event):
  if not self.scene_drag:return
  mode,x,y=self.scene_drag;dx,dy=event.x-x,event.y-y
  if mode=='orbit':self.scene_azimuth+=dx*.012;self.scene_elevation=max(math.radians(-78),min(math.radians(78),self.scene_elevation-dy*.012))
  else:self.scene_pan[0]+=dx;self.scene_pan[1]+=dy
  self.scene_drag=(mode,event.x,event.y);self.imu_rendered=None;self.draw_imu()
 def scene_wheel(self,event,direction=None):
  delta=direction if direction is not None else (1 if event.delta>0 else -1)
  self.scene_zoom=max(.35,min(3.,self.scene_zoom*(1.14 if delta>0 else 1/1.14)));self.imu_rendered=None;self.draw_imu()
 def draw_imu(self):
   c=self.imu_canvas;w=max(c.winfo_width(),1);h=max(c.winfo_height(),1);roll=self.roll;pitch=self.pitch;yaw=self.yaw
   state=(round(roll,3),round(pitch,3),round(yaw,3),self.cal_running,self.cal_step,self.trajectory_generation,len(self.trajectory),tuple(round(v,2) for v in self.trajectory_position),round(self.scene_azimuth,3),round(self.scene_elevation,3),round(self.scene_zoom,3),tuple(round(v,1) for v in self.scene_pan),w,h)
   if self.vehicle_mesh and state==self.imu_rendered:return
   self.imu_rendered=state;c.delete('all');trail=self.trajectory or [(0.,0.,0.)];extent=max(2.,max(max(abs(v) for v in p) for p in trail)*1.35);field_scale=min(w,h)*.32/extent*self.scene_zoom;field_cx=w*.52+self.scene_pan[0];field_cy=h*.65+self.scene_pan[1]
   def world(p):
    right,vertical,_=project_neu(p,self.scene_azimuth,self.scene_elevation);return field_cx+field_scale*right,field_cy-field_scale*vertical
   for i in range(-4,5):
    a=i*extent/4;c.create_line(*world((a,-extent,0)),*world((a,extent,0)),fill='#342519');c.create_line(*world((-extent,a,0)),*world((extent,a,0)),fill='#342519')
   c.create_line(*world((0,0,0)),*world((extent*.7,0,0)),fill='#8d6130',width=2,arrow='last');c.create_line(*world((0,0,0)),*world((0,extent*.7,0)),fill='#8d6130',width=2,arrow='last');c.create_text(*world((extent*.75,0,0)),text='N',fill='#d9b88e',anchor='sw');c.create_text(*world((0,extent*.75,0)),text='E',fill='#d9b88e',anchor='sw')
   c.create_oval(field_cx-5,field_cy-5,field_cx+5,field_cy+5,fill='#f5a23b',outline='');c.create_text(field_cx+9,field_cy+10,text='СТАРТ',fill='#f5a23b',anchor='nw',font=('Segoe UI',8,'bold'))
   if len(trail)>1:
    flat=[]
    for p in trail:flat.extend(world(p))
    c.create_line(*flat,fill='#f5a23b',width=2,dash=(6,4))
   model_origin=self.trajectory_position if self.trajectory else (0.,0.,0.);model_world=extent*.62
   def rotate(p):
    # Same body-FRD -> world-NED matrix as the ESP32, then NED down -> scene up.
    return frd_to_neu(p,roll,pitch,yaw)
   def point(p):
    north,east,up=rotate(p);north+=model_origin[0]+north*(model_world-1);east+=model_origin[1]+east*(model_world-1);up+=model_origin[2]+up*(model_world-1);right,vertical,depth=project_neu((north,east,up),self.scene_azimuth,self.scene_elevation);return field_cx+field_scale*right,field_cy-field_scale*vertical,depth
   def face(points,color,edge='#d7b66a'):
    q=[point(p) for p in points];c.create_polygon(*[v for p in q for v in p[:2]],fill=color,outline=edge);return sum(p[2] for p in q)/len(q)
   caption=(CAL_POSES[min(self.cal_step,5)][0]+' · фиксация положения') if self.cal_running else '3D-сцена'
   c.create_text(14,12,text=caption,fill='#efe2c3',anchor='nw',font=('Segoe UI',12,'bold'))
   yaw_text=(math.degrees(yaw)+180)%360-180;c.create_text(14,34,text=f'крен {math.degrees(roll):+.1f}°   тангаж {math.degrees(pitch):+.1f}°   курс (отн.) {yaw_text:+.0f}°',fill='#aab0b8',anchor='nw',font=('Consolas',10))
   if self.vehicle_mesh:
    shades=('#7d571f','#966825','#ae7d31','#c69649','#e6bc69')
    meshes=[(tri,shades[(i*7)%len(shades)]) for i,tri in enumerate(self.vehicle_mesh)]
    tail=(-.96,0,0);nose=(.96,0,0);ahead=(1.16,0,0)
   else:
    body=[(-.78,-.18,-.18),(.42,-.18,-.18),(.42,.18,-.18),(-.78,.18,-.18),(-.78,-.18,.18),(.42,-.18,.18),(.42,.18,.18),(-.78,.18,.18)]
    cone=[(.42,-.18,-.18),(.42,.18,-.18),(.42,.18,.18),(.42,-.18,.18),(.98,0,0)]
    nozzle=[(-.98,-.10,-.10),(-.78,-.18,-.18),(-.78,.18,-.18),(-.98,.10,-.10),(-.98,-.10,.10),(-.78,-.18,.18),(-.78,.18,.18),(-.98,.10,.10)]
    meshes=[([body[i] for i in f],col) for f,col in [((0,1,5,4),'#8e682d'),((1,2,6,5),'#c18d38'),((2,3,7,6),'#e0b55f'),((3,0,4,7),'#b9812e'),((4,5,6,7),'#f0ca73')]]
    meshes+=[([cone[i] for i in f],col) for f,col in [((0,1,4),'#a96e22'),((1,2,4),'#d9a64d'),((2,3,4),'#f0ca73'),((3,0,4),'#bd7d28')]]
    meshes+=[([nozzle[i] for i in f],col) for f,col in [((0,1,5,4),'#6e4b20'),((1,2,6,5),'#8c6429'),((2,3,7,6),'#b38843'),((3,0,4,7),'#765022')]]
    meshes+=[(p,col) for p,col in [([(-.62,-.18,-.18),(-.90,-.54,-.04),(-.18,-.40,-.04),(-.18,-.18,-.18)],'#9b7132'),([(-.62,.18,-.18),(-.90,.54,-.04),(-.18,.40,-.04),(-.18,.18,-.18)],'#c49a54'),([(-.62,-.18,.18),(-.90,-.45,.34),(-.18,-.34,.18),(-.18,-.18,.18)],'#b47f2e')]]
    tail=(-.98,0,0);nose=(.98,0,0);ahead=(1.22,0,0)
   # Canvas paints later polygons on top: draw farther camera-depth faces first.
   for pts,col in sorted(meshes,key=lambda m:sum(point(p)[2] for p in m[0])/len(m[0]),reverse=True):face(pts,col,'' if self.vehicle_mesh else '#d7b66a')
   tail=point(tail);nose=point(nose);ahead=point(ahead);c.create_line(tail[0],tail[1],ahead[0],ahead[1],fill='#d7b66a',dash=(4,3));c.create_text(ahead[0],ahead[1],text='▲',fill='#f2c35d',anchor='center',font=('Segoe UI',13,'bold'))
 def draw(self):
  alt_low,alt_high=self.nice_axis(list(self.altitude_history),min_span=12)
  g_low,g_high=self.nice_axis(list(self.g_history),min_span=1,zero_floor=True)
  speed_low,speed_high=self.nice_axis(list(self.speed_history),min_span=1,zero_floor=False)
  self.draw_chart(self.alt_canvas,'Высота ISA · 60 с',list(self.altitude_history),alt_low,alt_high,'#ed9e40','м')
  self.draw_chart(self.speed_canvas,'Вертикальная скорость · 60 с',list(self.speed_history),speed_low,speed_high,'#ed9e40','м/с')
  self.draw_chart(self.g_canvas,'Общая G-нагрузка · 60 с',list(self.g_history),g_low,g_high,'#f2c35d','G')
  self.draw_chart(self.pps_canvas,'Доставлено · 90 с',list(self.pps_history),0,55,'#f2c35d','пак/с')
  self.draw_chart(self.rssi_canvas,'RSSI · 60 с',list(self.rssi),-100,0,'#43d17a','dBm')
 def nice_axis(self,values,min_span=1,zero_floor=False):
  values=[v for v in values if isinstance(v,(int,float)) and math.isfinite(v)]
  if not values:return (0.,max(min_span,1.))
  low=min(values);high=max(values);span=max(high-low,min_span);pad=max(span*.12,min_span*.08);low=(0. if zero_floor else low-pad);high+=pad
  raw=max((high-low)/3.,min_span/3.)
  base=10**math.floor(math.log10(raw));step=next((m*base for m in (1,2,2.5,5,10) if m*base>=raw),10*base)
  low=0. if zero_floor else math.floor(low/step)*step;high=math.ceil(high/step)*step
  return (low,high if high>low else low+step)
 def draw_chart(self,c,title,values,low,high,color,unit):
  c.delete('all');w=max(c.winfo_width(),1);h=max(c.winfo_height(),1);left=38;right=9;top=24;bottom=20
  c.create_text(9,7,text=title,fill='#f4d7b3',anchor='nw',font=('Segoe UI',9,'bold'))
  for i in range(4):
   val=high-(high-low)*i/3;y=top+(h-top-bottom)*i/3;c.create_line(left,y,w-right,y,fill='#3b2819');c.create_text(3,y,text=f'{val:.0f}',fill='#ad8964',anchor='w',font=('Segoe UI',7))
  c.create_text(w-right,h-5,text='сейчас '+(f'{values[-1]:.1f} {unit}' if values else '—'),fill='#d9b88e',anchor='se',font=('Segoe UI',8))
  if len(values)<2:return
  target=max(2,int(w-left-right));step=max(1,(len(values)+target-1)//target);data=values[::step]
  if data[-1]!=values[-1]:data.append(values[-1])
  pts=[]
  for i,v in enumerate(data):
   x=left+i*(w-left-right)/(len(data)-1);y=top+(high-max(low,min(high,v)))*(h-top-bottom)/(high-low);pts.extend((x,y))
  c.create_line(*pts,fill=color,width=2)
if __name__=='__main__':App().mainloop()
