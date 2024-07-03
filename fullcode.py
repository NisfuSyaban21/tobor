import threading
import time
from ultralytics import YOLO
import cv2
import torch
import serial
import joblib
import pandas as pd

torch.cuda.set_device(0)

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)

def load_model_and_video():
    cap = cv2.VideoCapture(1)
    model = YOLO('best2juli2024.pt')
    return cap, model

def detect_objects_and_count(frame, model):
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.flip(frame, -1)
    result = model(frame, imgsz=320, conf=0.5, device='0')
    return frame, result

def count_objects(frame, result, classnames):
    warna_biru = []
    warna_merah = []
    kotak_silo = []
    koordinat_center = []
    x_center = 0

    for info in result:
        boxes = info.boxes
        for box in boxes:
            confidence = box.conf[0]
            Class = int(box.cls[0])
            if confidence > 0.6:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)
                if classnames[Class] == "silo":
                    kotak_silo.append([x1, y1, x2, y2])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                elif classnames[Class] == "biru":
                    warna_biru.append([x1, y1, x2, y2])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2) 
                elif classnames[Class] == "merah":
                    warna_merah.append([x1, y1, x2, y2])
                    koordinat_center.append([x_center, y_center])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0 ,0, 255), 2)
            
    return koordinat_center, warna_biru, warna_merah, x_center, kotak_silo

def count_objects_in_baskets(warna_biru, warna_merah, kotak_silo):
    result_text = []
    counts = []
    new_kotak_silo = sorted(kotak_silo, key=lambda x: x[0])
    for i, bbox in enumerate(new_kotak_silo):
        x1s, y1s, x2s, y2s = bbox
        biru_count = sum(1 for x1, y1, x2, y2 in warna_biru if x1 > x1s and x1 < x2s and y2 > y1s and y2 < y2s)
        merah_count = sum(1 for x1, y1, x2, y2 in warna_merah if x1 > x1s and x1 < x2s and y2 > y1s and y2 < y2s)
        counts.append((merah_count, biru_count))
        result_text.append("Keranjang {} - merah: {}, biru: {}".format(i + 1, merah_count, biru_count))
    return result_text, counts

def write_read(koordinat_center, kotak_pembatas):
    frame_center_x = (kotak_pembatas[0] + kotak_pembatas[2]) / 2
    frame_center_y = (kotak_pembatas[1] + kotak_pembatas[3]) / 2
    Vx = 0
    Vy = 0
    angle_deg = 0
    response = 0

    for x_center, y_center in koordinat_center:
        if (kotak_pembatas[0] <= x_center <= kotak_pembatas[2]) and (kotak_pembatas[1] <= y_center <= kotak_pembatas[3]):
            Vx = 0
            Vy = 0
            
        elif (x_center < 128):
            Vx = 0
            Vy = +30
            
        elif (x_center > 512):
            Vx = 0
            Vy = 0
            
        else:
            Vx = x_center - 320
            Vy = 480 - y_center
            

    combined_data = f"{Vx},{Vy}\n"
    arduino.write(combined_data.encode('utf-8'))
    time.sleep(0.05)
    response = arduino.readline().decode().strip()
    return response

def send_data_to_arduino_thread():
    global koordinat_center, kotak_pembatas
    while True:
        if koordinat_center and kotak_pembatas:
            response = write_read(koordinat_center, kotak_pembatas)
            print("Response from Arduino:", response)
        time.sleep(0.1)  # Menambahkan jeda waktu untuk mengontrol frekuensi pengiriman data

def decision(counts):
    counts_new = counts[:5]
    pengganti = {(0,0):int(1), (0,1):int(2),(1,0):int(4) ,(1,1):int(5) ,(2,0):int(),(0,2): int(0), (3,0): int(0), (0,3): int(0), (2,1): int(0),(1,2) : int(0), (0): int(0)}
    for i in range(len(counts_new)):
        if counts_new[i] in pengganti:
            counts_new[i] = pengganti[counts_new[i]]

    # Menambahkan nilai 0 untuk kolom yang hilang
    while len(counts_new) < 5:
        counts_new.append(0)

    model_decision = joblib.load('decision_tree_model1.joblib')
    # Membuat DataFrame dari data yang diberikan
    columns = ['Kolom 1', 'Kolom 2', 'Kolom 3', 'Kolom 4', 'Kolom 5']
    new_data = pd.DataFrame([counts_new], columns=columns)
    # Melakukan prediksi
    prediction = model_decision.predict(new_data)
    
    return prediction, counts_new

def decision_thread():
    global counts
    while True:
        if counts:
            prediction, counts_new = decision(counts)
            print("Prediction:", prediction)
            print("Counts new:", counts_new)
        time.sleep(1)  # Mengatur frekuensi eksekusi fungsi decision

def main_all():
    classnames = ['biru', 'merah', 'silo']
    cap, model = load_model_and_video()

    start_time = time.time()
    frame_counter = 0
    

    global koordinat_center, kotak_pembatas, counts
    koordinat_center = []
    kotak_pembatas = [200, 300, 450, 480]
    counts = []

    def capture_frames():
        nonlocal frame
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            time.sleep(0.01)

    def process_frames():
        nonlocal frame, frame_counter, start_time
        global koordinat_center, kotak_pembatas,counts
        while True:
            if frame is not None:
                frame_processed, result = detect_objects_and_count(frame, model)
                koordinat_center, warna_biru, warna_merah, x_center, kotak_silo = count_objects(frame_processed, result, classnames)
                result_text, counts = count_objects_in_baskets(warna_biru, warna_merah, kotak_silo)
                
                cv2.rectangle(frame_processed, (kotak_pembatas[0], kotak_pembatas[1]), (kotak_pembatas[2], kotak_pembatas[3]), (0, 255, 0), 2)
                for i, text in enumerate(result_text):
                    cv2.putText(frame_processed, text, (40, 200 + i * 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)

                frame_counter += 1
                if frame_counter >= 15:
                    end_time = time.time()
                    fps = round(15 / (end_time - start_time), 2)
                    cv2.putText(frame_processed, f'FPS: {fps}', (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
                    frame_counter = 0
                    start_time = time.time()

                cv2.imshow('frame', frame_processed)

                if cv2.waitKey(1) == ord('q'):
                    break

    frame = None
    capture_thread = threading.Thread(target=capture_frames)
    process_thread = threading.Thread(target=process_frames)
    send_to_arduino_thread = threading.Thread(target=send_data_to_arduino_thread)
    decision_thread_process = threading.Thread(target=decision_thread)

    capture_thread.start()
    process_thread.start()
    send_to_arduino_thread.start()
    decision_thread_process.start()

    capture_thread.join()
    process_thread.join()
    send_to_arduino_thread.join()
    decision_thread_process.join()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_all()
