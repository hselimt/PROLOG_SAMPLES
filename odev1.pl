% Belirtiler evet ya da hayir olarak girilir.

teshis(Ates, Oksuruk, NefesDarligi, Sonuc) :-
    belirtilen_sayi([Ates, Oksuruk, NefesDarligi], Evets),
    ( Evets >= 2 -> Sonuc = pozitif
    ; Evets =:= 1 -> Sonuc = supheli
    ; Sonuc = negatif ).

belirtilen_sayi([], 0).
belirtilen_sayi([evet|T], Say) :-
    belirtilen_sayi(T, Say2),
    Say is Say2 + 1.
belirtilen_sayi([hayir|T], Say) :-
    belirtilen_sayi(T, Say).

/* 
teshis(evet, evet, hayir, Sonuc).
Sonuc = pozitif.

teshis(hayir, evet, hayir, Sonuc).
Sonuc = supheli.

teshis(hayir, hayir, hayir, Sonuc).
Sonuc = negatif.
*/
