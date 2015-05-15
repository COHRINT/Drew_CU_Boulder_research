function plotGrid(target_truth,pos_estimate,agent_truth,space,search,time_list,accel_list,vel_est,vel_truth,quad)

xtx = target_truth.x_pos;
xty = target_truth.y_pos;
xt = pos_estimate.state(1);
yt = pos_estimate.state(2);
xt_truth = agent_truth.x_pos;
yt_truth = agent_truth.y_pos;
sense_radius = agent_truth.radius_of_detection;

persistent hplot;
persistent hag;
persistent hagr;
persistent htar;
persistent hag_true;
persistent hag_uncert;

if ~isempty(hplot)
    delete(hplot)
    figure(1)
else
    FigHandle = figure;
    uicontrol('Style','pushbutton','String','Very Close','Position',[170,5,70,20],'Callback',@veryClose);
    uicontrol('Style','pushbutton','String','Near','Position',[250,5,70,20],'Callback',@near);
    uicontrol('Style','pushbutton','String','Far','Position',[330,5,70,20],'Callback',@far);
 %   set(FigHandle, 'Position', [100, 100, 1200, 500]);
 %   subplot(3,2,2)
 
  %  subplot(3,2,4)

   % subplot(3,2,6)
end
%subplot(3,2,[1 3 5]);



hplot = pcolor(space.grid_x,space.grid_y,search.A); %colorbar;
shading interp;
if isempty(htar)
    set(gca,'color',[0 0 0]),hold on;
    htar = plot3(xtx,xty,0.02,'mx','MarkerSize',12,'LineWidth',2);                     %%nra: plotting target
    hagr = plot3(xt_truth,yt_truth,0.02,'co','MarkerSize',50*sense_radius,'LineWidth',1);
    hag = plot3(xt,yt,0.02,'wo','MarkerSize',12,'LineWidth',2);                %%nra: plotting agent, creating handle for loop updates
    hag_true = plot3(xt_truth,yt_truth,0.02,'y+','MarkerSize',12,'LineWidth',2);
    hag_uncert = plot3(xt,yt,0.02,'yo','MarkerSize',50*3*sqrt(pos_estimate.P(1)),'LineWidth',2);
    view(2)
else
    
    %           clf,hold on, grid
    set(hag,'XData',xt,'YData',yt)
    set(hagr,'XData',xt_truth,'YData',yt_truth)
    set(htar,'XData',xtx,'YData',xty)
    set(hag_true,'XData',xt_truth,'YData',yt_truth)
    set(hag_uncert,'XData',xt,'YData',yt,'MarkerSize',50*3*sqrt(pos_estimate.P(1)))
    %title(['time: ',num2str(floor(t))],'FontSize',14)
end

if size(time_list,2) <= 5000
    %subplot(3,2,2)
  %  plot(time_list,accel_list)
   % axis([0 50 -1 1]);
    
    %subplot(3,2,4)
    %plot(time_list,[vel_est(:,1),vel_truth(:,1)])
    %axis([0 50 -1 1]);
    
   % subplot(3,2,6)
    %plot(time_list,[vel_est(:,2),vel_truth(:,2)])
    %axis([0 50 -1 1]);
    
else
    %subplot(3,2,2)
    %time_lim = size(time_list,2);
    %plot(time_list(time_lim-5000:time_lim),accel_list(time_lim-5000:time_lim,:));
    %axis([time_lim/100 - 50,time_lim/100,-1,1]);

    %subplot(3,2,4)
    %plot(time_list(time_lim-5000:time_lim),[vel_est(time_lim-5000:time_lim,1),vel_truth(time_lim-5000:time_lim,1)]);
    %axis([time_lim/100 - 50,time_lim/100,-1,1]);

   % subplot(3,2,6)
    %plot(time_list(time_lim-5000:time_lim),[vel_est(time_lim-5000:time_lim,2),vel_truth(time_lim-5000:time_lim,2)]);
    %axis([time_lim/100 - 50,time_lim/100,-1,1]);
 
end


end

function near(hObject,eventdata,handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
	display('near');
elseif button_state == get(hObject,'Min')
	display('up');
end
end

function veryClose(hObject,eventdata,handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of togglebutton1
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
	display('very close');
elseif button_state == get(hObject,'Min')
	display('up');
end
end

function far(hObject,eventdata,handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of togglebutton1
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
	display('far');
elseif button_state == get(hObject,'Min')
	display('up');
end
end
