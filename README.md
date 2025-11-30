# PID-and-Thrust-Allocation
Autonomous boat control simulation featuring PID controller and Thrust Allocation using C++ and Eigen library.
Bu proje, C++ ve Eigen kütüphanesi kullanılarak geliştirilmiş bir otonom tekne kontrol simülasyonudur. Proje şunları içerir:

PID Kontrolcü: Hedef açıya ulaşmak için hata hesaplaması yapar.

Thrust Allocation (İtki Dağıtımı): Hesaplanan torku matris işlemleriyle (Eigen) 3 farklı motora (Sol, Sağ, Baş) kuvvet olarak dağıtır.

Basit Fizik Motoru: Teknenin eylemsizliğini ve sürtünmesini simüle eder.

PWM Dönüştürücü: Newton cinsinden kuvveti motor PWM sinyallerine çevirir.
