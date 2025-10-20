% -------------------------
% MEDICAL DIAGNOSIS
% -------------------------
% Symptoms are entered as yes or no.

diagnosis(Fever, Cough, Breathlessness, Result) :-
    count_symptoms([Fever, Cough, Breathlessness], YesCount),
    ( YesCount >= 2 -> Result = positive
    ; YesCount =:= 1 -> Result = suspicious
    ; Result = negative ).

count_symptoms([], 0).
count_symptoms([yes|T], Count) :-
    count_symptoms(T, Count2),
    Count is Count2 + 1.
count_symptoms([no|T], Count) :-
    count_symptoms(T, Count).

/* 
diagnosis(yes, yes, no, Result).
Result = positive.

diagnosis(no, yes, no, Result).
Result = suspicious.

diagnosis(no, no, no, Result).
Result = negative.
*/


% -------------------------
% STUDENT PASS/FAIL CODE
% -------------------------
student(zeynep, 50, 60, 70, 3, 0).
student(ahmet, 50, 60, 90, 6, 0).
student(mehmet, 20, 55, 40, 2, 30).

calculate_grade(Midterm, Final, Makeup, Grade) :-
    ( Makeup > 0 ->
        Grade is Midterm * 4 / 10 + Makeup * 6 / 10;
        Grade is Midterm * 4 / 10 + Final * 6 / 10
    ).

pass_and_grade(Student, true, Grade) :-
    student(Student, Midterm, Final, _, Absences, Makeup), Absences =< 5,
    calculate_grade(Midterm, Final, Makeup, Grade), Grade >= 50.

pass_and_grade(Student, false, Grade) :-
    student(Student, Midterm, Final, _, Absences, Makeup),
    calculate_grade(Midterm, Final, Makeup, Grade),
    ( Absences > 5 ; Grade < 50 ).


% -------------------------
% FAMILY TREE
% -------------------------
/* pass_and_grade(StudentName, Pass, Grade) */
child(ali, ayse, ahmet).
child(ayse, fatma, mehmet).
child(veli, ayse, ahmet).

mother(Mother, Child) :- child(Child, Mother, _). % ignore father
father(Father, Child) :- child(Child, _, Father). % ignore mother
sibling(X, Y) :- child(X, Mother, Father), child(Y, Mother, Father), X \= Y.

/* mother(ayse, Child) 
   sibling(ali, Sibling)
   father(ahmet, Child)
*/


% -------------------------
% LOGIC GATES
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
