clear all
clc
n=50;
red =35;
w_guess=zeros(1,50);
v_guess=zeros(1,50);

w_guess(1:(n-red))=w_guess(red+1:n);
    v_guess(1:(n-red))=v_guess(red+1:n);
    ck=n-red;
    n=35;
    ck+1
    n
    n-ck-1
    w_guess(ck+1:n)=w_guess(ck)*ones(1,n-ck);
    v_guess(ck+1:n)=v_guess(ck)*ones(1,n-ck);