%testing code
 t = sym('t', [1 4]);
 syms fin real;
 k = tril(ones(4,4));
 fin = k*t'; 
 for i = 1:4
 val(i,:) =fin(:)'.*k(i,:) ;
 end
 
 