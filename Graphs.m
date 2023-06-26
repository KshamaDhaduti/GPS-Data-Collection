bag1=rosbag("Static.bag");
bag2=rosbag("walk.bag");
bsel1=select(bag1,"Topic","custom_message");
bsel2=select(bag2,"Topic","custom_message");
msgStruct1=readMessages(bsel1,'DataFormat','struct');
msgStruct2=readMessages(bsel2,'DataFormat','struct');
utm_easting_stat=cellfun(@(m) double(m.UtmEasting),msgStruct1);
utm_easting_lin=cellfun(@(m) double(m.UtmEasting),msgStruct2);
utm_northing_stat=cellfun(@(m) double(m.UtmNorthing),msgStruct1);
utm_northing_lin=cellfun(@(m) double(m.UtmNorthing),msgStruct2);
alt1=cellfun(@(m) double(m.Altitude),msgStruct1);
alt2=cellfun(@(m) double(m.Altitude),msgStruct2);

subplot(2,1,1)
plot(utm_easting_stat-min(utm_easting_stat), utm_northing_stat-min(utm_northing_stat), 'r+')
xlabel("UTM Easting (in meters)")
ylabel("UTM Northing (in meters)")
title("GPS Static Data")

%UTM_E vs UTM_N
subplot(2,1,2)  %Linear
plot(utm_easting_lin-min(utm_easting_lin),utm_northing_lin-min(utm_northing_lin))
hold on
p=polyfit(utm_easting_lin-min(utm_easting_lin),utm_northing_lin-min(utm_northing_lin),1);
f=polyval(p,utm_easting_lin-min(utm_easting_lin));
plot(utm_easting_lin-min(utm_easting_lin),f,'-')
xlabel("UTM Easting (in meters)")
ylabel("UTM Northing (in meters)")
title("GPS Linrear")

figure
%Grapg 2
%subplot(2,1,1)

%subplot(2,1,2)
%title("GPS Error vs Time")


%TEime vs UTM easting Stat
subplot(2,1,1)
p=polyfit([1:length(utm_easting_stat)],utm_easting_stat-min(utm_easting_stat),1);
f_utm_east_stat=polyval(p,[1:length(utm_easting_stat)]);
plot(utm_easting_stat-min(utm_easting_stat))
xlabel("Time")
ylabel("UTM Easting (in meters)")
title("Static UTM-Easting Data")

%Time vs UTMeasting LINEAR
%{
subplot(2,2,2)
plot(utm_easting_lin)
hold on
p=polyfit(utm_easting_lin-min(utm_easting_lin),utm_northing_lin-min(utm_northing_lin),1);
f=polyval(p,utm_easting_lin-min(utm_easting_lin));
plot(utm_easting_lin-min(utm_easting_lin),f,'-')
xlabel("Time")
ylabel("UTM Easting (in meters)")
title("Linear motion UTM-Easting Data")
%}


%Time vs UTM northing stat
subplot(2,1,2)
p=polyfit([1:length(utm_northing_stat)],utm_easting_stat-min(utm_northing_stat),1);
f_utm_north_stat=polyval(p,[1:length(utm_northing_stat)]);
plot(utm_northing_stat-min(utm_northing_stat))
xlabel("Time")
ylabel("UTM Northing (in meters)")
title("Static UTM-Northing Data")

%{
%Time vs UTMNorthing LINEAR
%subplot(2,2,4)
plot(utm_northing_lin) 
minx = 3272013 %min(utm_easting_stat)
%miny = 4689688 %min(utm_northing_stat)
xlabel("Time")
ylabel("UTM Northing (in meters)")
title("Linear motion UTM-Northing Data")
figure
%}

figure
%Dont touch code is good
subplot(2,1,1)
plot(alt1)
xlabel("Time")
ylabel("Altitude (in meters)")
title("Static Altitude Data")

subplot(2,1,2)
plot(alt2)
xlabel("Time")
ylabel("Altitude (in meters)")
title("Line motion Altitude Data")


%Error Plots
utm_east_stat_err=f_utm_east_stat-utm_easting_stat
utm_north_stat_err=f_utm_north_stat-utm_northing_stat
figure
subplot(2,1,1)
plot(abs(utm_east_stat_err-min(utm_east_stat_err)),'o')
title('Absolute values of UTM Easting stationary point error values from best fit line values')
xlabel('Time(seconds)')
ylabel('Absolute value of error(metres)')
subplot(2,1,2)
plot(abs(utm_north_stat_err-min(utm_north_stat_err)),'o')
title('Absolute values of UTM Northing stationary point error values from best fit line values')
xlabel('Time(seconds)')
ylabel('Absolute value of error(metres)')



