import threading
import time
from ultralytics import YOLO
import cv2
import torch
import serial
import joblib
import pandas as pd
import math

# Mengatur perangkat
device = 'cuda' if torch.cuda.is_available() else 'cpu'
#device = 'cpu' if torch.cuda.is_available() else 'cuda'
#print(f"Perangkat yang digunakan: {device}")

# Inisialisasi serial untuk Arduino
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
time.sleep(2)

def load_model_and_video():
    cap = cv2.VideoCapture(0)
    # cap.set(cv2.CAP_PROP_BRIGHTNESS,0.5)
    model = YOLO('150epochseptember.pt')
    #model.to(device)  # Pindahkan model ke perangkat yang tepat
    #print(f"Model berjalan di perangkat: {model.device}")  # Verifikasi perangkat
    return cap, model

def detect_objects_and_count(frame, model):
    frame = cv2.resize(frame, (700, 450))
    frame = cv2.flip(frame, -1)
    result = model(frame, imgsz=320, conf=0.6, device=device, stream=True)  # Gunakan device yang benar
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
            if confidence > 0.7:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)
                if classnames[Class] == "silo":
                    kotak_silo.append([x1, y1, x2, y2])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f'{classnames[Class]}:{confidence:.2f}'
                    cv2.putText(frame, label, (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)
                elif classnames[Class] == "merah":
                    warna_merah.append([x1, y1, x2, y2])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    label = f'{classnames[Class]}:{confidence:.2f}'
                    cv2.putText(frame, label, (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)
                elif classnames[Class] == "biru":
                    warna_biru.append([x1, y1, x2, y2])
                    koordinat_center.append([x_center, y_center])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255 ,0, 0), 2)
                    label = f'{classnames[Class]}:{confidence:.2f}'
                    cv2.putText(frame, label, (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)

                #label = f'{classnames[Class]}:{confidence:.2f}'
                #cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_PLAIN, 0.5, (255, 255, 255), 1)

    return koordinat_center, warna_biru, warna_merah, x_center, kotak_silo

def count_objects_in_baskets(warna_biru, warna_merah, kotak_silo):
    result_text = []
    counts = []
    new_kotak_silo = sorted(kotak_silo, key=lambda x: x[0])
    global jumlah_silo
    jumlah_silo = len(new_kotak_silo)
    for i, bbox in enumerate(new_kotak_silo):
        x1s, y1s, x2s, y2s = bbox
        biru_count = sum(1 for x1, y1, x2, y2 in warna_biru if (x1+x2)/2 > x1s and (x1+x2)/2 < x2s and (y1+y2)/2 > y1s and (y1+y2)/2 < y2s)
        merah_count = sum(1 for x1, y1, x2, y2 in warna_merah if (x1+x2)/2 > x1s and (x1+x2)/2 < x2s and (y1+y2)/2 > y1s and (y1+y2)/2 < y2s)
        counts.append((merah_count, biru_count))
        result_text.append("Keranjang {} - merah: {}, biru: {}".format(i + 1, merah_count, biru_count))
    return result_text, counts, new_kotak_silo

def mengirimkan_data():
    global response, prediction, jumlah_silo

    response = arduino.readline().decode().strip()
    if (response == "readyred") and (jumlah_silo == 3):
        combined_data = str(prediction)
        arduino.write(combined_data.encode())
        # time.sleep(0.05)

# Menambahkan jeda waktu untuk mengontrol frekuensi pengiriman data

def mengirimkan_data_thread():
    global frame_processed
    while True:
        if frame_processed is not None:
            mengirimkan_data()
            # print("respon dari arduino ", response)

def decision(counts):
    counts_new = counts[:5]
    pengganti = {
        (0, 0): int(2), (0, 1): int(3), (1, 0): int(4), (1, 1): int(5), (2, 0): int(1),
        (0, 2): int(0), (3, 0): int(0), (0, 3): int(0), (2, 1): int(0), (1, 2): int(0)
    }
    counts_new = [pengganti.get(c, 0) for c in counts_new]
    while len(counts_new) < 5:
        counts_new.append(0)

    model_decision = joblib.load('model_decision_tree.joblib')
    columns = ['Silo1', 'Silo2', 'Silo3', 'Silo4', 'Silo5']
    new_data = pd.DataFrame([counts_new], columns=columns)
    predictionss = model_decision.predict(new_data)
    prediction = predictionss[0]
    return prediction, counts_new

def decision_thread():
    global frame_processed
    while True:
        if frame_processed is not None:
            prediction, counts_new = decision(counts)
            # print("Prediction:", prediction)
        # time.sleep(1)  # Mengatur frekuensi eksekusi fungsi decision

def main_all():
    classnames = ['biru', 'merah', 'silo']
    cap, model = load_model_and_video()
    start_time = time.time()
    frame_counter = 0
    frame_counter_1 = 0

    global koordinat_center, counts_new, prediction, counts, warna_merah, frame, errorX, errorY, errorDeg, frame_processed, response, Cx, Cy, distancee
    global jumlah_silo

    koordinat_center = []
    counts = []
    frame = None
    errorX = 0
    errorY = 0
    errorDeg = 0
    frame_processed = None
    response = None
    warna_merah = []
    Cx = 0
    Cy = 0
    distancee = 0
    prediction = 0
    counts_new = []

    def capture_frames():
        global frame
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            time.sleep(0.01)

    def process_frames():
        global koordinat_center, counts, warna_merah, frame_processed, frame, response, Cx, Cy, distancee, prediction, counts_new
        nonlocal frame_counter, start_time

        while True:
            if frame is not None:
                frame_processed, result = detect_objects_and_count(frame, model)
                koordinat_center, warna_biru, warna_merah, x_center, kotak_silo = count_objects(frame_processed, result, classnames)
                result_text, counts, new_kotak_silo = count_objects_in_baskets(warna_biru, warna_merah, kotak_silo)

                prediction, counts_new = decision(counts)
                # cv2.line(frame_processed,(360,0),(360,480),(255,0,0),2)
                # cv2.line(frame_processed,(230,0),(230,480),(255,0,0),2)

                # cv2.putText(frame_processed, f'counts: {counts}', (40, 350), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
                # cv2.putText(frame_processed, f'counts_new: {counts_new}', (40, 400), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
                cv2.rectangle(frame_processed, (9, 9), (322, 149), (0, 0, 0), -1)
                for i, text in enumerate(result_text):
                    cv2.putText(frame_processed, text, (40, 70 + i * 15), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)

                frame_counter += 1
                if frame_counter >= 15:
                    end_time = time.time()
                    fps = round(15 / (end_time - start_time), 2)
                    cv2.putText(frame_processed, f'FPS: {fps}', (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
                    frame_counter = 0
                    start_time = time.time()
                    
                #frame_counter_1 += 1    
                #cv2.putText(frame_processed, f'frame ke-: {frame_counter}', (100, 33), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)

                # Call the tracking function
                cv2.putText(frame_processed, f'prediksi: {prediction}', (150, 22), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
                # cv2.putText(frame_processed, f'warna_merah: {warna_merah}', (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
                cv2.putText(frame_processed, f'respon arduino: {response}', (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
                # response = mengirimkan_data(errorX, errorY, errorDeg)
                cv2.imshow('frame', frame_processed)
                if cv2.waitKey(1) == ord('q'):
                    break

    # Inisialisasi thread
    capture_thread = threading.Thread(target=capture_frames)
    process_thread = threading.Thread(target=process_frames)
    mengirimkan = threading.Thread(target=mengirimkan_data_thread)
    # decision_thread_process = threading.Thread(target=decision_thread)

    # Mulai thread
    capture_thread.start()
    process_thread.start()
    mengirimkan.start()
    # decision_thread_process.start()

    # Tunggu thread selesai
    capture_thread.join()
    process_thread.join()
    mengirimkan.join()
    # decision_thread_process.join()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_all()
