from ultralytics import YOLO
import cv2
import math
import time
import joblib
import pandas as pd

def load_model_and_video():
    # Membuka video
    cap = cv2.VideoCapture(0)
    # Menginisialisasi model YOLO
    model = YOLO('best4april24.pt')
    return cap, model

def detect_objects_and_count(cap, model):
    # Membaca setiap frame dari video
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    # Mendeteksi objek menggunakan model YOLO
    result = model(frame, stream=True, conf=0.55)
    return frame, result

def count_objects(result, classnames):
    warna_biru = []
    warna_merah = []
    kotak_silo = []

    for info in result:
        boxes = info.boxes
        for box in boxes:
            confidence = box.conf[0]
            confidence = math.ceil(confidence * 100)
            Class = int(box.cls[0])
            if confidence > 55:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                if classnames[Class] == "silo":
                    kotak_silo.append([x1, y1, x2, y2])
                elif classnames[Class] == "biru":
                    warna_biru.append([x1, y1, x2, y2])
                elif classnames[Class] == "merah":
                    warna_merah.append([x1, y1, x2, y2])
            
                

    return warna_biru, warna_merah, kotak_silo

def count_objects_in_baskets(warna_biru, warna_merah, kotak_silo):
    result_text = []
    counts = []
    new_kotak_silo = sorted(kotak_silo, key=lambda x: x[0])
    for i, bbox in enumerate(new_kotak_silo):
        x1s, y1s, x2s, y2s = bbox
        biru_count = sum(1 for x1, y1, x2, y2 in warna_biru if x1 > x1s and x1 < x2s and y2 > y1s-10 and y2 < y2s)
        merah_count = sum(1 for x1, y1, x2, y2 in warna_merah if x1 > x1s and x1 < x2s and y2 > y1s-10 and y2 < y2s)
        counts.append((merah_count, biru_count))
        result_text.append("Keranjang {} - merah: {}, biru: {}".format(i + 1, merah_count, biru_count))
    print(counts)
    return result_text, counts

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
    
    return prediction,counts_new

def main_all():
    classnames = ['biru', 'merah', 'silo']
    cap, model = load_model_and_video()

    while True:
        frame, result = detect_objects_and_count(cap, model)
        warna_biru, warna_merah, kotak_silo = count_objects(result, classnames)
        result_text,counts = count_objects_in_baskets(warna_biru, warna_merah, kotak_silo)
        prediction,counts_new = decision(counts)
        for i, text in enumerate(result_text):
            cv2.putText(frame, text, (40, 400 + i * 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
        
        cv2.putText(frame, "masukkan bola ke silo : "+str(prediction), (40, 400 + 75), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
        cv2.imshow('frame', frame)
        
        print(counts_new)
        print(prediction)


        


        if cv2.waitKey(1) == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()
    return counts,counts_new

if __name__ == "__main__":
    main_all()
