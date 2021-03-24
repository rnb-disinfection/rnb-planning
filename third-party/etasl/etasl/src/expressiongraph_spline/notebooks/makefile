all:
	@cat readme.md

clean:
	rm -f spline1.dat      spline1_outp.dat  spline2_norm.dat  spline3_outp.dat
	rm -f spline1_inp.dat  spline2_inp.dat   spline2_outp.dat  spline4_outp.dat
	rm -f lua_scripts2.dot  lua_scripts2.png  lua_scripts.dot  lua_scripts.png
	rm -f circle.csv input.csv input2.csv input.csv.bak output.csv debug_init.m
	rm -f tmp.lua read_csv.pyc
	rm -f examples.html lua_scripts.html analysing_ur_example1.html Motion_model.html
	
save:
	cat examples.ipynb | python clear_ipynb.py > examples.clean_ipynb
	cat lua_scripts.ipynb | python clear_ipynb.py > lua_scripts.clean_ipynb
	cat analysing_ur_example1.ipynb | python clear_ipynb.py > analysing_ur_example1.clean_ipynb 
	cat Motion_model.ipynb | python clear_ipynb.py > Motion_model.clean_ipynb
create:
	cp examples.clean_ipynb examples.ipynb
	cp lua_scripts.clean_ipynb lua_scripts.ipynb
	cp analysing_ur_example1.clean_ipynb analysing_ur_example1.ipynb 
	cp Motion_model.clean_ipynb Motion_model.ipynb

html:
	ipython nbconvert --to html --template full examples.ipynb
	ipython nbconvert --to html --template full lua_scripts.ipynb
	ipython nbconvert --to html --template full analysing_ur_example1.ipynb
	ipython nbconvert --to html --template full Motion_model.ipynb
