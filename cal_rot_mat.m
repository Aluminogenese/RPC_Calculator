dcm_demo=dcmeci2ecef('IAU-2000/2006',[2021 5 15 17 40 53]);

ac_time=load('data\utc_imgTime.txt');
[L,~]=size(ac_time(:,1));
for i = 1:L
    dcm=dcmeci2ecef('IAU-2000/2006',ac_time(i,:));
    line_dcm=reshape(dcm',1,[]);  %列为主顺序展平 故先将dcm转置

    if i==1   
        fid = fopen('data\J2000_2_wgs84.txt','w'); 
     else
         fid = fopen('data\J2000_2_wgs84.txt','a'); 
    end
    for data = line_dcm(:)
        fprintf(fid,"%f ",data);
    end
    fprintf(fid,"\n");
    fclose(fid);
end
