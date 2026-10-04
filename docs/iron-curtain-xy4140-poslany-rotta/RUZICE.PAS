{$A-,B-,D-,E+,F-,I+,L-,N+,O-,R-,S-,V+}
{$M 1024,0,0}
program RUZICE;
const
   xb=750;  yb=1050; ax=750; ay=400;  bx=1313; by=725;
   cx=1313; cy=1375; dx=750; dy=1700; ex=187;
   ey=1375; fx=187;  fy=725;  d=0.05; i=50;
var
   x,y,xa,ya,xc,yc,x1,y1,x2,y2,x3,y3:real;
                                 p,j:byte;
                               kolik:boolean;
                              soubor:text;
{****************************************}
procedure vypocti;
begin
   x1:=xa/10; y1:=ya/10; x2:=xb/10; y2:=yb/10; x3:=xc/10; y3:=yc/10;
end;
{****************************************}
procedure zapis(a,b,c,d:real);
begin
   writeln(soubor,'PA',trunc(a*40),' ',trunc(b*40)
                 ,'PD',trunc(c*40),' ',trunc(d*40),'PU');
end;
{****************************************}
procedure nova_poloha;
begin
    x:=(x2-x1)*d+x1;  y:=(y2-y1)*d+y1;
   x2:=(x3-x2)*d+x2; y2:=(y3-y2)*d+y2;
   x3:=(x1-x3)*d+x3; y3:=(y1-y3)*d+y3;
   x1:=x; y1:=y;
end;
{****************************************}
procedure prvni;
begin
   vypocti;
   j:=1;
   repeat
     zapis(x1,y1,x2,y2);
     nova_poloha;
     zapis(x2,y2,x1,y1);
     nova_poloha;
     j:=j+2;
   until (j>=i);
end;
{****************************************}
procedure druha;
begin
   vypocti;
   j:=1;
   repeat
     zapis(x3,y3,x2,y2);
     nova_poloha;
     zapis(x2,y2,x3,y3);
     nova_poloha;
     j:=j+2;
   until (j>=i);
end;
{****************************************}
procedure skoc;
begin
   case kolik of
     false:prvni;
      true:druha;
   end;
end;
{****************************************}
begin
  writeln('Start programu ... ');
  assign(soubor,'RUZICE.PLT');  rewrite(soubor);
  kolik:=false;
  for p:=1 to 2 do
    begin
      if p=1 then writeln(soubor,'SP1') else writeln(soubor,'SP2');
      xa:=ax; ya:=ay; xc:=bx; yc:=by; skoc;
      xa:=cx; ya:=cy; skoc;
      xc:=dx; yc:=dy; skoc;
      xa:=ex; ya:=ey; skoc;
      xc:=fx; yc:=fy; skoc;
      xa:=ax; ya:=ay; skoc;
      kolik:=true;
    end;
  close(soubor);
writeln('Konec programu');
end.