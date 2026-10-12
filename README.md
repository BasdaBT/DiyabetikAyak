# STM32 Diyabetik Ayak Erken Risk İzleme Prototipi

Bu proje, iki ayaktan alınan termal ve plantar basınç verilerini karşılaştırmak
için geliştirilen STM32F407VGT6 tabanlı bir gömülü sistem prototipidir. MLX90640
termal sensöründen gelen 32 x 24 sıcaklık matrisi ile 10 kanallı FSR basınç
ölçümü birlikte değerlendirilir ve sonuçlar UART üzerinden raporlanır.

> Bu çalışma bir mühendislik prototipidir. Tıbbi cihaz veya klinik tanı aracı
> değildir; ürettiği sonuçlar sağlık kararı vermek için kullanılmamalıdır.

## Mevcut özellikler

- MLX90640 termal sensörünün I2C üzerinden okunması
- 10 FSR kanalının ADC ve DMA ile sürekli örneklenmesi
- İki ayak için sıralı termal ve basınç ölçüm akışı
- Sıcaklık verisinde ortalama ve standart sapma tabanlı aykırı değer süzme
- Ayaklar arasındaki sıcaklık farkı ve basınç asimetrisine göre risk skoru
- `basla`, `devam` ve `rst` komutlarını işleyen kesme tabanlı UART arayüzü
- Durum göstergesi için STM32F407 Discovery kartı LED'leri

## Sistem akışı

1. Sistem MLX90640 bağlantısını kontrol eder ve FSR örneklemesini başlatır.
2. `basla` komutu veya kullanıcı butonu ilk ayak ölçümünü başlatır.
3. Termal çerçeve ve 10 FSR kanalının ortalama değerleri kaydedilir.
4. `devam` komutu ikinci ayak ölçümünü başlatır.
5. Sıcaklık farkı ve basınç asimetrisi hesaplanarak UART üzerinden risk skoru
   gönderilir.

## Donanım ve yazılım

| Bileşen | Görev |
| --- | --- |
| STM32F407VGT6 | Ana denetleyici |
| MLX90640 | 32 x 24 temassız termal görüntüleme |
| 10 x FSR | Plantar basınç örnekleme |
| ADC1 ve DMA | FSR kanallarının sürekli ölçümü |
| I2C1 | MLX90640 haberleşmesi |
| USART3 | Komut ve sonuç aktarımı |
| STM32CubeIDE | Derleme, hata ayıklama ve programlama |

## Risk değerlendirmesi

Mevcut firmware kural tabanlı bir prototip algoritması kullanır. Sıcaklık farkı
ve etkin FSR bölgelerindeki ortalama basınç asimetrisi, toplam risk skoruna
katkı sağlar. Yapay zekâ tabanlı sınıflandırıcı entegrasyonu projenin devam eden
çalışmaları arasındadır; bu depodaki mevcut sürümde tamamlanmış bir AI modeli
bulunmamaktadır.

## Proje yapısı

- `Core/Src/main.c` - ölçüm durum makinesi, UART komutları ve risk hesabı
- `Core/Src/MLX90640_API.c` - MLX90640 sıcaklık hesaplama kütüphanesi
- `Core/Src/MLX90640_I2C_Driver.c` - sensör haberleşme katmanı
- `Core/Src/adc.c`, `dma.c`, `i2c.c`, `usart.c` - çevre birimi ayarları
- `USB_DEVICE/` - STM32 USB Device yapılandırması
- `diaabetic1.ioc` - STM32CubeMX proje yapılandırması

## Derleme

1. Depoyu klonlayın.
2. `diaabetic1.ioc` dosyasını STM32CubeIDE ile açın veya projeyi mevcut bir
   STM32CubeIDE projesi olarak içe aktarın.
3. Hedefin `STM32F407VGT6` olarak seçildiğini doğrulayın.
4. Projeyi derleyip karta programlayın.
5. USART3 bağlantısını açarak sistem komutlarını ve ölçüm sonuçlarını izleyin.

## Durum

Proje geliştirme aşamasındadır. Sensör kalibrasyonu, farklı kullanıcılarla
tekrarlanabilirlik testleri, güvenli skor sınırları ve AI modeli doğrulaması
tamamlanmadan klinik kullanım hedeflenmemektedir.
