#include<iostream>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;
class PID_Controller{
private:
    float kp,ki,kd;
    float previous_error,total_error;

public:
    PID_Controller(float p,float i,float d){
    kp=p;//o anki hatayý ne kadar ciddiye alacaðýný gösterir. artarsa agresiflik artar.
    ki=i;//hafýzada hata birikiyosa bunu ne kadar ciddiye alacaðýdýr
    kd=d;//geleceðe göre ne kadar sert frenlemeli
    previous_error=0;
    total_error=0;
    }

    float calculate(float hedef, float current,float dt){
        //hata
        float hata=hedef-current;
        //p=oransal
        float p=kp*hata;
        //I ÝNTEGRAL HESABI
        total_error+=hata*dt;
        float i=ki*total_error;
        //d türev hesabý
        float derivative=(hata-previous_error)/dt;
        float d=kd*derivative;
        previous_error=hata;

        float output=p + i+ d;
        return output;
    }
};
class SimpleBoatPysics{
private:
    //teknenin burnunun baktýðý açý
    float currentAngle=0.0;
    //teknenin dönme hýzý
    float angularVelocity=0.0;
    //teknenin eylemsizliði
    float inertia=2.0;
    //sürtünme katsayýsý
    float drag=0.9;
public:
    void update(float tork, float dt){
        float AngularAcc=tork/inertia;
        angularVelocity = angularVelocity + (AngularAcc*dt);
        angularVelocity = angularVelocity * drag;
        currentAngle = currentAngle + (angularVelocity * dt);
    }
    float getAngle(){
        return currentAngle;
    }
};
class ThrustAllocator{
private:
    Matrix3f B;
    Matrix3f B_inv;
    //uzaklýklar metre
    float d=0.5;
    float L=1.0;
public:
    ThrustAllocator(){
    //1. matris
    //satýrlar Surge(Fx),Sway(Fy),Yaw(Mz)
    B<< 1, 1, 0,//fx:sol motor ve sað motor ileri iter
        0, 0, 1,//fy:sadece baþ itici yana iter
        d, -d,L;//Mz:tork etkileri sap poizitif,sol negatif,baþ poizitif

    //terisini al
    B_inv=B.inverse();
    }
    //istenen kuvvetleri newton a dönüþtürme fonksiyonu
    Vector3f calculateForces(float Fx,float Fy,float Mz){
    //ÝSTEK VEKTÖRÜ TAU YANÝ TORK
    Vector3f tau;
    tau<<Fx,Fy,Mz; //1. ifade ilk satýr 2. ifade ikinci satýr, 3. ifade üçüncü satýr.

    //temel denklem
    Vector3f u=B_inv*tau;

    return u;//u(0) sol u(1) sað u(2) ön motor
    }
    //yardýmcý fonksiyon
    int forceToPWM(float force){
        // Basit lineer haritalama
        // 0N -> 1500 (Dur)
        // 50N -> 1900 (Tam Ýleri)
        // -50N -> 1100 (Tam Geri)
        float max_force=50;
        int pwm=1500;
        if (force > 0) {
            pwm = 1500 + (int)((force / max_force) * 400);
        } else {
            pwm = 1500 + (int)((force / max_force) * 400);
        }

        // Sýnýrlandýrma (1100 - 1900 arasý)
        if(pwm > 1900) pwm = 1900;
        if(pwm < 1100) pwm = 1100;

        return pwm;
    }
};
int main(){
    ThrustAllocator allocator;
    SimpleBoatPysics myBoat;
    PID_Controller myPID(4.0,0.0,1.0);
    float hedef_aci=90.0;
    float dt=0.1;
    int step=0;
    while(true){
        //sensöt verisi
        float current=myBoat.getAngle();
        //ne kadar tork lazým
        float torkKomutu =myPID.calculate(hedef_aci,current,dt);
        //Torku motorlara kuvvet olarak daðýtýyorum
        Vector3f kuvvetler = allocator.calculateForces(0.0, 0.0, torkKomutu);
        // 4. PWM ÇEVÝRÝMÝ: Newton -> PWM
        // Vektörün içinden elemanlarý (0), (1), (2) ile çekip çeviriyoruz.
        int pwm_sol = allocator.forceToPWM(kuvvetler(0));
        int pwm_sag = allocator.forceToPWM(kuvvetler(1));
        int pwm_bas = allocator.forceToPWM(kuvvetler(2));
        //tekneye yukle
        myBoat.update(torkKomutu,dt );
       cout << "Aci: " << current
             << " | Tork: " << torkKomutu
             << " | SOL: " << pwm_sol
             << " | SAG: " << pwm_sag
             << " | BAS: " << pwm_bas << endl;
        // Döngü çok hýzlý akmasýn diye basit bir frenleme (isteðe baðlý)
        // Gerçek kodda burasý sleep fonksiyonu olur.
        step++;
        if(step > 1000) break; // 1000 adým sonra dursun (Test için)
    }
return 0;
}
