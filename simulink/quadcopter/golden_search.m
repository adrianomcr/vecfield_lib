function s_star = golden_search(al, bl, p, t, curva)
    gold = 0.6180339887;

	s_a = bl - gold*(bl-al);
	s_b = al + gold*(bl-al);

	[ra,~,~] = my_curve(s_a, t, curva);
	dist_vec = p-ra; %[0] = x[0]-ra[0]; dist_vec[1] = x[1]-ra[1]; dist_vec[2] = x[2]-ra[2];
	func_a = norm(dist_vec);%sqrt(dist_vec[0]*dist_vec[0] + dist_vec[1]*dist_vec[1] + dist_vec[2]*dist_vec[2]);
    
    [rb,~,~] = my_curve(s_b, t, curva);
	dist_vec = p-rb; %[0] = x[0]-ra[0]; dist_vec[1] = x[1]-ra[1]; dist_vec[2] = x[2]-ra[2];
	func_b = norm(dist_vec);%sqrt(dist_vec[0]*dist_vec[0] + dist_vec[1]*dist_vec[1] + dist_vec[2]*dist_vec[2]);

	k = 0;
	while(bl-al > 0.000000000001)
        k = k+1;
%         if (k==100)
%             break
%         end
        
        
        if(func_a > func_b)

			al = s_a;
			s_a = s_b;
			s_b = al + gold*(bl-al);

            [ra,~,~] = my_curve(s_a, t, curva);
            func_a = norm(p-ra);

            [rb,~,~] = my_curve(s_b, t, curva);
            func_b = norm(p-rb);

        else

			bl = s_b;
			s_b = s_a;
			s_a = bl - gold*(bl-al);
            
            [ra,~,~] = my_curve(s_a, t, curva);
            func_a = norm(p-ra);

            [rb,~,~] = my_curve(s_b, t, curva);
            func_b = norm(p-rb);
            
        end

        
        
        
    end


	s_star = (al+bl)/2;



end % function