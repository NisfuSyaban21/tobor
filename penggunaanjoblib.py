from sklearn.datasets import load_iris
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import joblib
import pandas as pd

# Membaca file CSV ke dalam DataFrame pandas
data = pd.read_csv('tabel_keputusan1.csv')

# Memisahkan fitur (kolom 1-5) dan target (kolom Keputusan)
x = data.iloc[:, :-1]  # Fitur
y = data.iloc[:, -1]   # Target

# Bagi data menjadi data latih dan data uji
X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)

# Inisialisasi model Decision Tree
model = DecisionTreeClassifier()

# Latih model dengan data latih
model.fit(X_train, y_train)

# Simpan model ke dalam file menggunakan joblib
joblib.dump(model, 'decision_tree_model1.joblib')

# Lakukan prediksi dengan data uji
y_pred = model.predict(X_test)

# Hitung akurasi model
accuracy = accuracy_score(y_test, y_pred)
print(f'Akurasi Decision Tree: {accuracy:.2f}')
