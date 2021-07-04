		#set terminal epslatex size 3,3 color dashed
		#set output "histogramcolor.tex"
    unset key
		set multiplot layout 2,1 
		max_val = 3000.0
		max_color = 100.0
		tics =  max_val/5
		color_tics = max_color/5
    set grid

    	set multiplot
		set arrow from .1, 2.1 to screen 0.4, 0.4 front lt 3

		set size 1,0.75
		set origin 0, 0.25
		set bmargin 0
		set rmargin 4
		set lmargin 10
		set tmargin 1
		set boxwidth 1.8 absolute
		set format cb "%3.1f"
		set xrange [0:max_val]
		set yrange [*:1600]
		set xlabel "Distance [cm]"
		set ylabel "Number of Occurrences" offset 0, 0
		#set ytics 0,2000 
		set style fill solid noborder
		set xtics tics 
		bin_width = 0.1
		bin_number(x) = floor(x/bin_width)
		rounded(x) = bin_width * ( bin_number(x) + 0.5 )
		set table 'tmp.txt'
		#plot "datdistance.txt" using (rounded($4)):(4) smooth frequency with boxes
		unset table
		unset datafile separator
		plot "hist.txt" using 1:2:(hsv2rgb((1-$1/max_color)*(240.0/360.0),1,1)) with boxes lc rgb variable

		set origin .4, .4
		set size .5,.5
		clear
		unset key
		unset grid
		unset object
		unset arrow
		unset xtics
		unset xlabel
		unset ylabel

		set xtics 50
		set bmargin 1
		set tmargin 1
		set lmargin 3
		set rmargin 1
		plot[0:150] "hist.txt" using 1:2:(hsv2rgb((1-$1/max_color)*(240.0/360.0),1,1)) with boxes lc rgb variable

		
    
    set view map
		set xtics format "%g"
		set style function pm3d
		set palette color
		set size 1.193,0.2
		set bmargin 1
		set lmargin 10
		set origin -0.088,0
		f(x)=(1-x/max_color)*(240.0/360.0)
		#set cbrange [f(0):f(max_val)] # [0:1]
		set cbrange [0:1] # [0:1]
		set xrange [0:max_color]
		set yrange [*:*]
		set xtics color_tics
		set cbtics 0.2
		set format cb "%3.1f"
		unset ztics
		unset ytics
		#set samples 101
		set samples 300
		set isosamples 2
		unset key
		unset colorbox
		set palette model HSV defined ( 0 0 1 1, 1 1 1 1 )

		splot f(x)

    #set terminal x11




