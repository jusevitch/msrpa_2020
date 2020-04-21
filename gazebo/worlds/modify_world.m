% data=fileread('city.world');
% data= regexp(data, '\r\n', 'split');
% istarget = startsWith(data, ',pose>')



%Open file
fread=fopen('city_block8.world','r');
fwrite=fopen('city_block_W19.world','w');

% fread=fopen('city.world','r')
% fwrite=fopen('city_new.world','w');
lx1=-62.7;
lx2=109.5;
ly1=-81.2;
ly2=126.2;


%pos_offset=[-4*(lx2-lx1)+lx2+lx1,ly2+ly1,0]';
%pos_offset=[-1*(lx2-lx1),-(ly2-ly1),0]';
pos_offset=[-4*(lx2-lx1)+(lx2+lx1),2*(ly2-ly1)+ly2+ly1,0]';
alpha=pi;
rotMat=[cos(alpha), sin(alpha), 0; -sin(alpha), cos(alpha), 0; 0, 0, 1];

%World suffix
worldSuffix='_W19';


while ~feof(fread)
    fline=fgets(fread);
    %Check for pose
    num=textscan(fline,'<pose> %f %f %f %f %f %f </pose>');
    
    %check for road name
    roadName=textscan(fline,'<road name="%s">');
    
    %check for sidewalk name
    sidewalkName= textscan(fline,'<model name="%s">');
    
    %check for other model names
    modelName=textscan(fline,'<name>%s</name>');
    
    if length(num{1})>0
        num2=[];
        if isnan(num{1})
            num=[]
            for i=1:6
                fline=fgets(fread);
                num=[num, textscan(fline,'%f')]
            end
            fline=fgets(fread); %To remove the extra </pose>
        end
        pos=[num{1},num{2},num{3}]';
        pos_new=rotMat*pos+pos_offset;
        num2=[pos_new',num{4},num{5},num{6}+alpha];
        
        fprintf(fwrite,'      <pose> %f %f %f %f %f %f </pose>\n',num2);
    elseif length(roadName{1})>0
        roadName{1}{1}(end-1:end)=[];
        str=append(roadName{1}{1},worldSuffix);
        fprintf(fwrite,'    <road name="%s">\n',str);
        %write the line with width information
        fline=fgets(fread);
        fprintf(fwrite,fline);
        %Update the end points of the road
        for i=1:2
            fline=fgets(fread);
            num=textscan(fline,'<point>%f %f %f</point>');
            pos=[num{1},num{2},num{3}]';
            pos_new=rotMat*pos+pos_offset;
            fprintf(fwrite,'    <point>%f %f %f</point>\n', pos_new);
        end
        
    elseif length(sidewalkName{1})>0
        sidewalkName{1}{1}(end-1:end)=[];
        str=append(sidewalkName{1}{1},worldSuffix);
        fprintf(fwrite,'    <model name="%s">\n',str);
    elseif length(modelName{1})>0
        modelName{1}{1}(end-6:end)=[];
        str=append(modelName{1}{1},worldSuffix);
        if strcmp(modelName{1}{1},'Gazebo/Residential')
            fprintf(fwrite,fline);
        elseif strcmp(modelName{1}{1},'GasStation/Diffuse')
            fprintf(fwrite,fline);
        elseif strcmp(modelName{1}{1},'vrc/asphalt') 
            fprintf(fwrite,fline);
        else
            fprintf(fwrite,'    <name>%s</name>\n',str);
        end
        
    else
        fprintf(fwrite,fline);
    end
end
fclose(fread);
fclose(fwrite);