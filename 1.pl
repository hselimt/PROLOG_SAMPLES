% -------------------------
% ÖĞRENCİ GEÇER/KALIR KODU
% -------------------------
ogrenci(zeynep, 50, 60, 70, 3, 0).
ogrenci(ahmet, 50, 60, 90, 6, 0).
ogrenci(mehmet, 20, 55, 40, 2, 30).

not_hesapla(Vize, Final, Butunleme, Puan) :-
    ( Butunleme > 0 ->
        Puan is Vize * 4 / 10 + Butunleme * 6 / 10;
        Puan is Vize * 4 / 10 + Final * 6 / 10
    ).

gecer_ve_puan(Ogrenci, true, Puan) :-
    ogrenci(Ogrenci, Vize, Final, _, Devamsizlik, Butunleme), Devamsizlik =< 5,
    not_hesapla(Vize, Final, Butunleme, Puan), Puan >= 50.

gecer_ve_puan(Ogrenci, false, Puan) :-
    ogrenci(Ogrenci, Vize, Final, _, Devamsizlik, Butunleme),
    not_hesapla(Vize, Final, Butunleme, Puan),
    ( Devamsizlik > 5 ; Puan < 50 ).


% -------------------------
% SOY AĞACI KODU
% -------------------------
/* gecer_ve_puan(OgrenciAd, Gecer, Puan) */
cocuk(ali, ayse, ahmet).
cocuk(ayse, fatma, mehmet).
cocuk(veli, ayse, ahmet).

anne(Anne, Cocuk) :- cocuk(Cocuk, Anne, _).
baba(Baba, Cocuk) :- cocuk(Cocuk, _, Baba).
kardes(X, Y) :- cocuk(X, Anne, Baba), cocuk(Y, Anne, Baba), X \= Y.

/* anne(ayse, Cocuk) 
   kardes(ali, Kardes)
   baba(ahmet, Cocuk)
*/


% -------------------------
% MANTIK DEVRESİ KAPILARI
% -------------------------
not(1, 0).
not(0, 1).

and(1, 1, 1).
and(1, 0, 0).
and(0, 1, 0).
and(0, 0, 0).

or(1, 1, 1).
or(1, 0, 1).
or(0, 1, 1).
or(0, 0, 0).

/* not(1, X).
   and(1, 0, X).
   or(0, 1, X).
*/
