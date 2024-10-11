function createfigure(XData1, YData1, YData2, Parent1)
%CREATEFIGURE(XData1, YData1, YData2, Parent1)
%  XDATA1:  line xdata
%  YDATA1:  line ydata
%  YDATA2:  line ydata
%  PARENT1:  text parent

%  由 MATLAB 于 16-Aug-2024 12:18:50 自动生成

% 创建 figure
figure('WindowState','maximized','Tag','ScopePrintToFigure','Color',[1 1 1],...
    'OuterPosition',[-6.33333333333333 24.3333333333333 1454.66666666667 944]);

% uicontainer 当前不支持代码生成，请输入正确输入语法对应的 'doc uicontainer'
% 可以使用 GUIDE 来为 uicontainer 生成代码。有关详细信息，请输入 'doc guide'
% uicontainer(...);

% uicontainer 当前不支持代码生成，请输入正确输入语法对应的 'doc uicontainer'
% 可以使用 GUIDE 来为 uicontainer 生成代码。有关详细信息，请输入 'doc guide'
% uicontainer(...);

% uipanel 当前不支持代码生成，请输入正确输入语法对应的 'doc uipanel'
% 可以使用 GUIDE 来为 uipanel 生成代码。有关详细信息，请输入 'doc guide'
% uipanel(...);

% 创建 axes
axes1 = axes('Tag','DisplayAxes1_RealMag');
hold(axes1,'on');
colororder([0.0666666666666667 0.443137254901961 0.745098039215686;0.866666666666667 0.329411764705882 0;0.929411764705882 0.694117647058824 0.125490196078431;0.52156862745098 0.0862745098039216 0.819607843137255;0.231372549019608 0.666666666666667 0.196078431372549;0.184313725490196 0.745098039215686 0.937254901960784;0.819607843137255 0.0156862745098039 0.545098039215686]);

% 创建 hgtransform
hgtransform('HitTest','off','Matrix',[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]);

% 创建 hgtransform
hgtransform('HitTest','off','Matrix',[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]);

% 创建 line
line(XData1,YData1,'DisplayName','\phi','Tag','DisplayLine1','LineWidth',2,...
    'Color',[1 0 0]);

% 创建 line
line(XData1,YData2,'DisplayName','\theta','Tag','DisplayLine2',...
    'LineWidth',2,...
    'Color',[0 0.447058823529412 0.741176470588235]);

% 创建 xlabel
xlabel(' ','FontName','Times New Roman');

% 取消以下行的注释以保留坐标区的 X 范围
% xlim(axes1,[0 8.00000000000005]);
% 取消以下行的注释以保留坐标区的 Y 范围
% ylim(axes1,[-0.122429167602133 0.110565977066589]);
% 取消以下行的注释以保留坐标区的 Z 范围
% zlim(axes1,[-1 1]);
box(axes1,'on');
hold(axes1,'off');
% 设置其余坐标区属性
set(axes1,'ClippingStyle','rectangle','FontName','Times New Roman',...
    'FontSize',8,'GridAlpha',0.4,'GridColor',[0 0 0],'TickLabelInterpreter',...
    'none','XGrid','on','YGrid','on');
% 创建 legend
legend1 = legend(axes1,'show');
set(legend1,'Units','pixels','Interpreter','none','FontSize',30,...
    'EdgeColor',[0 0 0]);

% 创建 text
text('Tag','TimeOffsetStatus','Parent',Parent1,'Units','pixels',...
    'VerticalAlignment','bottom',...
    'FontSize',8,...
    'Position',[0 0 0],...
    'Visible','on');

