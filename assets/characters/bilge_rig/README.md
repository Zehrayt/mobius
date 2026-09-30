# Bilge parçalı 2D rig

Bu klasör sabit kameralı yürüyüş için 16 saydam PNG içerir: baş, gövde,
kalça, örgü, iki üst kol, iki ön kol, iki el, iki üst bacak, iki alt bacak
ve iki ayakkabı. Özgün `../bilge_happy.png` değiştirilmez.

Baş/yüz **özgün PNG'nin RGB piksellerinden** alınır; yalnızca başı ve boynu
ayıran alfa maskesi uygulanır. Yüzün oranları çizimde tek ölçekle korunur.
Gövde, örgü ve uzuv atlası, aynı Bilge referansı verilerek yerleşik
`imagegen` aracıyla hazırlandı. Gizli omuz/kalça yüzeyleri ve eklem bağlantı
payları bu aşamada tamamlandı. Kullanılan tam istem `generation_prompt.txt`,
ham çıktı `atlas.png` dosyasındadır. Atlasın yeniden üretilmesi gerekmez.

`python3 demo/prepare_bilge_rig.py` atlası adlandırılmış dosyalara ayırır,
başın özgün piksellerini alır ve `rig.json` dosyasını üretir. Bu komut
ayarları yeniden yazar; kalıcı paketleme değişikliklerini `PARTS` içinde yapın.
Normal video üretimi paketleme komutunu çalıştırmaz.

`rig.json` içindeki `anchor`/`end`, kırpılmış PNG üzerinde normalize
bağlantı noktalarıdır. `width_px`, görünüm genişliğini belirler. Bacak ve
kol parçalarının iki ucu, her karede iskeletin ilgili iki eklemine tam
eşlenir. Baş, eller, kalça ve ayakkabılar tek bağlantıyla taşınır/döner.
Çizim sırası `demo/bilge_walk_skinned.py:ORDER` içindedir.

Ayak eklemi taban temas noktasıdır. Ayakkabının alfa sınırının en altı bu
noktaya hizalanır; ayakkabının bilek bağlantısı da paçanın görsel uç
noktasıdır. Bu ofset, orijinal iskelet verisine yazılmaz. Destekte ayakkabı
dönmez; salınımda küçük parmak ucu kalkışı uygulanır. Kenar yumuşatma
nedeniyle yarım opak taban pikseli zeminden en fazla 1 piksel yukarıdadır.

Bu açıdaki test için eksik parça yok. Bu paket önden/üç çeyrek görünümü
koruyan 2D kukla animasyonudur; arkadan görünüş veya tam yan profil
dönüşü için ayrı kafa, saç, gövde, el ve ayakkabı açıları gerekecektir.
