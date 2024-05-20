import itertools
import csv

# Mendefinisikan nilai yang akan dimasukkan ke dalam tabel
nilai = [5, 4, 3, 2, 1]

# Membuat semua kemungkinan kombinasi nilai untuk setiap kolom menggunakan itertools.product
kombinasi_nilai = list(itertools.product(nilai, repeat=5))

# Menambahkan kolom "Keputusan" berdasarkan nilai dari 5 baris sebelumnya
tabel_keputusan = []
for kombinasi in (kombinasi_nilai):
    if 5 in kombinasi:
        keputusan = nilai[4 - kombinasi.index(5)]
    elif 4 in kombinasi:
        keputusan = nilai[4 - kombinasi.index(4)]
    elif 3 in kombinasi:
        keputusan = nilai[4 - kombinasi.index(3)]
    elif 2 in kombinasi:
        keputusan = nilai[4 - kombinasi.index(2)]
    else:
        keputusan = nilai[4 - kombinasi.index(1)]
    tabel_keputusan.append(kombinasi + (keputusan,))


# Mencetak header tabel
print("|", " | ".join([f"Kolom {i}" for i in range(1, 6)]), "|")
print("|" + "|".join(["-" for _ in range(33)]) + "|")
for kombinasi in tabel_keputusan:
    print("{:^8} | {:^8} | {:^8} | {:^8} | {:^8} | {:^9}".format(*kombinasi))


header = ["Kolom 1", "Kolom 2", "Kolom 3", "Kolom 4", "Kolom 5", "Keputusan"]

# Nama file CSV untuk disimpan
nama_file = "tabel_keputusan1.csv"

# Menyimpan tabel keputusan ke dalam file CSV
with open(nama_file, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)
    # Menulis setiap baris tabel keputusan
    writer.writerows(tabel_keputusan)

print(f"File {nama_file} berhasil disimpan.")
