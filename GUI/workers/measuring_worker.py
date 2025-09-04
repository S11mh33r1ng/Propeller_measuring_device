import app_globals
from plot.canvas import Canvas
from PyQt5.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot
import time, datetime, os, csv, math, statistics

class MeasuringWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int, str)
    input_data = pyqtSignal(str)
    requestStart = pyqtSignal()
    requestStop = pyqtSignal()
    stopTimer = pyqtSignal()

    def __init__(self, shared_data):
        super(MeasuringWorker, self).__init__()
        self.shared_data = shared_data
        self._running = False
        self.stopTimer.connect(self.stop_timer)
        self.measuring_stopped = True
        self.e_stop = False
        self.rpm = 0
        self.cnv = Canvas()
        self.counter = 0
        self.custom_trajectory = False
        self.omega_avg = 0
        self.arspd_avg = 0
        self.aoa_avg = 0
        self.aoss_avg = 0
        self.v_tan = 0
        self.v_rad = 0
        self.v_axial = 0
        self.meas_timer = QTimer(self)
        self.meas_timer.timeout.connect(self.measure)
        self.x_goal = 0
        self.x_prev = 0
        self.first_line_done = False
        self.header_added = False
        self.x_normalized_mm = 0
        self.goal_reached = False
        self.b = 0
        self.time_delay = 0
        self.send_once = False
        self.cycle_time = 1
        self.current_x_target = 0
        self.current_y_target = 0
        self.data = ''
   
    def start_measuring(self):
        self.goal_reached = False
        app_globals.app_globals.window.tare_done = False
        self.wait = 0
        if app_globals.app_globals.window.counter == 0:
            app_globals.app_globals.window.today_dt = datetime.datetime.today().strftime('%d-%m-%Y-%H:%M:%S')
            self.parent_dir = "/home/siim/Desktop/logid/"
            app_globals.app_globals.window.path = os.path.join(self.parent_dir, app_globals.app_globals.window.today_dt)
            os.mkdir(app_globals.app_globals.window.path)
            app_globals.window.csvfile = "log" + app_globals.window.today_dt + ".csv"
            self.header = ['Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 'Omega(rad/s) ' 'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s)']
            self.header_added = False
        app_globals.window.cnv.clear_plots()
        self.aoss_a = []
        self.aoa_a = []
        self.arspd_a = []
        self.omega_a = []
        app_globals.window.label13.clear()
        app_globals.window.label15.clear()
        app_globals.window.label17.clear()
        app_globals.window.label19.clear()
        app_globals.window.label65.clear()
        app_globals.window.label67.clear()
        app_globals.window.label69.clear()
        app_globals.window.measure.setEnabled(False)
        app_globals.window.Y_move.setEnabled(False)
        app_globals.window.measuring_stopped = False
        app_globals.window.test_progress.setMaximum(app_globals.window.sweep_count.value())
        self.first_line_done = False
        app_globals.window.radius_mm = format(float((app_globals.window.prop.value()*25.4*(1 + self.shared_data.safety_over_prop/100))/2),'.1f')
        self.radius_steps = format((float(app_globals.window.radius_mm) * float(self.shared_data.ratio)),'.0f')
        app_globals.window.x_goal = int(app_globals.window.mm_to_steps(self.shared_data.x_center) - int(self.radius_steps))
        app_globals.window.progress.setValue(1)
        app_globals.window.sendData('BeaconON')
        time.sleep(3)
        app_globals.window.sendData('tare')
        while (app_globals.window.tare_done == False):
            print("andurite nullimine")
        app_globals.window.progress.setValue(5)
        app_globals.window.sendData('streamStart')
        time.sleep(2)
        print(app_globals.window.radius_mm)
        throttle = int(1000 + (app_globals.window.throttle.value() * 10))
        start = f'start|{throttle}'
        app_globals.window.sendData(start)
        time.sleep(10)
        app_globals.window.progress.setValue(10)
        self.x_normalized_mm = 0
        self.b = 0
        self.current_x_target = 0
        self.current_y_target = 0
        self.send_once = True
        app_globals.window.sendData('m|%d|%d|%d|%d' %(((self.shared_data.x_center - 3) * self.shared_data.ratio), (app_globals.window.Y_pos.value() * self.shared_data.ratio), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value()/3)))
        time.sleep(2)
        app_globals.window.sendData('m|%d|%d|%d|%d' %(((self.shared_data.x_center) * self.shared_data.ratio), (app_globals.window.Y_pos.value() * self.shared_data.ratio), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value()/3)))
        time.sleep(3)
        self.meas_timer.start(self.cycle_time)  # Measure every 100ms
        
    def start_measuring_after_first(self):
        app_globals.window.tare_done = False
        app_globals.window.cnv.clear_plots()
        self.aoss_a = []
        self.aoa_a = []
        self.arspd_a = []
        self.omega_a = []
        app_globals.window.label13.clear()
        app_globals.window.label15.clear()
        app_globals.window.label17.clear()
        app_globals.window.label19.clear()
        app_globals.window.label65.clear()
        app_globals.window.label67.clear()
        app_globals.window.label69.clear()
        app_globals.window.measure.setEnabled(False)
        app_globals.window.Y_move.setEnabled(False)
        app_globals.window.measuring_stopped = False
        app_globals.window.test_progress.setMaximum(app_globals.window.sweep_count.value())
        self.first_line_done = False
        app_globals.window.radius_mm = format(float((app_globals.window.prop.value()*25.4*(1 + self.shared_data.safety_over_prop/100))/2),'.1f')
        self.radius_steps = format((float(app_globals.window.radius_mm) * float(self.shared_data.ratio)),'.0f')
        app_globals.window.x_goal = int(app_globals.window.mm_to_steps(self.shared_data.x_center) - int(self.radius_steps))
        time.sleep(10)
        if app_globals.window.counter == 0:
            app_globals.window.progress.setValue(1)
            app_globals.window.sendData('tare')
            while (app_globals.window.tare_done == False):
                print("ootan")
        app_globals.window.progress.setValue(5)
        app_globals.window.sendData('streamStart')
        time.sleep(2)
        throttle = int(1000 + (app_globals.window.throttle.value() * 10))
        start = f'start|{throttle}'
        app_globals.window.sendData(start)
        time.sleep(10)
        app_globals.window.progress.setValue(10)
        self.x_normalized_mm = 0
        self.b = 0
        self.x_prev = 0
        self.send_once = True
        self.goal_reached == False
        self.current_x_target = 0
        self.current_y_target = 0
        self.data = ""
        print(self.data)
        self.measure()
            
    def measure(self):
        if not self._running:
            self.meas_timer.stop()
            #print("lõpetan")
            return
        if app_globals.window.custom_trajectory == False and self.send_once == True:
            list_of_x_targets.append(app_globals.window.x_goal)
            list_of_y_targets.append(0)
            app_globals.window.sendData('m|%d|%d|%d|%d' %(list_of_x_targets[0], (list_of_y_targets[0] + (app_globals.window.Y_pos.value() * self.shared_data.ratio)), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value()/3)))
            self.send_once = False
        if app_globals.window.custom_trajectory == True and self.b == 0 and self.send_once == True:
            try:
                app_globals.window.sendData('m|%d|%d|%d|%d' % (list_of_x_targets[self.b], ((app_globals.window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value() / 3)))     
                self.current_x_target = list_of_x_targets[self.b]
                self.current_y_target = int(((app_globals.window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]))
                self.send_once = False
            except:
                print("0 write failed")
        if app_globals.window.custom_trajectory == True and 0 < self.b <= (len(list_of_x_targets) - 1) and self.send_once == True:
            try:
                app_globals.window.sendData('m|%d|%d|%d|%d' % (list_of_x_targets[self.b], ((app_globals.window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value() / 3)))     
                self.current_x_target = list_of_x_targets[self.b]
                self.current_y_target = int(((app_globals.window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]))
                self.send_once = False
            except:
                print("write failed")
        #print(self.x_normalized_mm)
        if (int(self.x_normalized_mm) == 0 and self.first_line_done == False):
            aoss_zero = format(app_globals.window.aoss_abs,'.2f')
            aoa_zero = format(app_globals.window.aoa_abs,'.2f')
            omega_zero = app_globals.window.omega
            arspd_zero = format(app_globals.window.airspeed,'.2f')
            v_tan_zero = format(float(self.arspd_avg) * math.sin(math.radians(float(self.aoa_avg))),'.2f')
            v_rad_zero = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.sin(math.radians(float(self.aoss_avg))),'.2f')
            v_axial_zero = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.cos(math.radians(float(self.aoss_avg))),'.2f')
            data_zero = str(str(format(app_globals.window.prop.value(),'.1f'))+" "+str(self.x_normalized_mm)+" "+str(app_globals.window.Y_pos.value())+" "+str(app_globals.window.trq_current)+" "+str(app_globals.window.thr_current)+" "+str(omega_zero)+" "+str(arspd_zero)+" "+str(aoa_zero)+" "+str(aoss_zero)+" "+str(v_tan_zero)+" "+str(v_rad_zero)+" "+str(v_axial_zero))
            with open(os.path.join(app_globals.window.path,app_globals.window.csvfile), 'a') as f:
                w = csv.writer(f)
                if not self.header_added:
                    w.writerow(self.header)
                    self.header_added = True
                w.writerow([data_zero])
            self.first_line_done = True
            self.x_prev = self.x_normalized_mm
        if (int(self.x_normalized_mm) - int(self.x_prev) < int(self.shared_data.x_delta) and self.first_line_done == True):
            self.aoss_a.append(float(app_globals.window.aoss_abs))
            self.aoa_a.append(float(app_globals.window.aoa_abs))
            self.arspd_a.append(float(app_globals.window.airspeed))
            self.omega_a.append(float(app_globals.window.omega))
        if (int(self.x_normalized_mm) - int(self.x_prev) >= int(self.shared_data.x_delta) and self.first_line_done == True):
            self.aoss_avg = format(statistics.mean(self.aoss_a), '.2f')
            self.aoa_avg = format(statistics.mean(self.aoa_a),'.2f')
            self.arspd_avg = format(statistics.mean(self.arspd_a),'.2f')
            if float(self.arspd_avg) > 100.00:
                self.arspd_avg = 0.00
            self.omega_avg = format(statistics.mean(self.omega_a),'.2f')
            self.v_tan = format(float(self.arspd_avg) * math.sin(math.radians(float(self.aoa_avg))),'.2f')
            self.v_rad = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.sin(math.radians(float(self.aoss_avg))),'.2f') 
            self.v_axial = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.cos(math.radians(float(self.aoss_avg))),'.2f') 
            self.data = str(str(format(app_globals.window.prop.value(),'.1f'))+" "+str(self.x_normalized_mm)+" "+str(app_globals.window.steps_to_mm(app_globals.window.y_current_steps))+" "+str(app_globals.window.trq_current)+" "+str(app_globals.window.thr_current)+" "+str(self.omega_avg)+" "+str(self.arspd_avg)+" "+str(self.aoa_avg)+" "+str(self.aoss_avg)+" "+str(self.v_tan)+" "+str(self.v_rad)+" "+str(self.v_axial))
            #print(self.data)
            with open(os.path.join(app_globals.window.path,app_globals.window.csvfile), 'a') as f:
                w = csv.writer(f)
                if not self.header_added:
                    w.writerow(self.header)
                    self.header_added = True
                w.writerow([self.data])
            self.x_prev = self.x_normalized_mm
            app_globals.window.update_plot_ax1(self.x_normalized_mm, self.omega_avg, self.arspd_avg, self.aoa_avg, self.aoss_avg, app_globals.window.trq_current, app_globals.window.thr_current)
            self.aoss_a.clear()
            self.aoa_a.clear()
            self.arspd_a.clear()
            self.omega_a.clear()
        fg = self.shared_data.x_center - (int(app_globals.window.steps_to_mm(app_globals.window.x_goal)))
        if app_globals.window.meas_data_running:
            self.x_normalized_mm = format(float(self.shared_data.x_center) - (int(app_globals.window.steps_to_mm(app_globals.window.x_current_steps))),'.0f')
            #print(self.x_normalized_mm)
            #print(type(self.x_normalized_mm))
            x_progress = round((int(self.x_normalized_mm)/int(fg))*90,0)
            app_globals.window.progress.setValue(10 + int(x_progress))
            #print(app_globals.window.x_current_steps)
            if (app_globals.window.x_current_steps <= app_globals.window.x_goal and self.goal_reached == False):
                #print(app_globals.window.x_goal)
                self.goal_reached = True
                app_globals.window.counter = app_globals.window.counter + 1
                app_globals.window.test_progress.setValue(app_globals.window.counter)
                app_globals.window.meas_data_running = False
                #print("eesmärk täidetud")
                self.check_progress(app_globals.window.counter, app_globals.window.x_current_steps)
#             if self.goal_reached:
#                 app_globals.window.counter = app_globals.window.counter + 1
#                 app_globals.window.test_progress.setValue(app_globals.window.counter)
#                 self.goal_reached = False
#                 app_globals.window.meas_data_running = False
#                 self.check_progress(app_globals.window.counter, app_globals.window.x_current_steps)
            if (app_globals.window.x_current_steps == self.current_x_target and app_globals.window.y_current_steps == self.current_y_target and app_globals.window.custom_trajectory == True and self.goal_reached == False):
                self.b = self.b + 1
                if self.b < len(list_of_x_targets):
                    self.send_once = True
                if self.b >= len(list_of_x_targets):
                    self.send_once = False
                #print(self.b)
                pass
            
        else:
            self.x_normalized_mm = 0
        
    def check_progress(self, cycles_done, x_curr):
        #print(cycles_done)
        if cycles_done == app_globals.window.sweep_count.value():
            app_globals.window.measuring_stopped = True
            app_globals.window.come_back()
            self.stop_timer()
            app_globals.window.process_data()
        else:
            self.goal_reached = False
            time.sleep(1)
            app_globals.window.sendData('stop')
            time.sleep(2)
            app_globals.window.sendData('j|%d|0|%d|%d' %(x_curr, app_globals.window.jog_speed.value(), app_globals.window.jog_speed.value()))
            time.sleep(3)
            app_globals.window.sendData('center')
            time.sleep(5)
            app_globals.window.sendData('j|%d|%d|%d|%d' %((self.shared_data.x_center * self.shared_data.ratio), (app_globals.window.Y_pos.value() * self.shared_data.ratio), app_globals.window.jog_speed.value(), app_globals.window.jog_speed.value()))
            self.start_measuring_after_first()

    def stop_measuring(self):
        self._running = False
        self.stopTimer.emit()

    @pyqtSlot()
    def stop_timer(self):
        self.meas_timer.stop()
        self._running = False
        print("Measurement stopped")
    
    @pyqtSlot()
    def run(self):
        self._running = True
        self.start_measuring()