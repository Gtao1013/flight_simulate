static_data = xlsread('气动数据库','定常');
dynamic_data = xlsread('气动数据库','动导数');

H=static_data(:,1);
Ma=static_data(:,2);
alpha=static_data(:,3);
beta=static_data(:,4);

len=length(static_data);   %数据行数
data_H=[];          %高度的数据
for i=1:len
    if static_data(i,1)>0
        data_H=[data_H,static_data(i,1)];
    end
end
data_H=sort(unique(data_H))*1000;          %获取高度的数据，去掉重复项，由小到大排列,化成米
num_H=length(data_H);                 %高度的个数

data_Ma=[];                           %马赫数的数据
for i=1:len
    if static_data(i,2)>0
        data_Ma=[data_Ma,static_data(i,2)];
    end
end
data_Ma=sort(unique(data_Ma));        %获取马赫数的数据，去掉重复项，由小到大排列
num_Ma=length(data_Ma);               %马赫数的个数

original_data_alpha=sort(unique(static_data(:,3)));  %获取原始攻角数据

% 找到最接近0的负攻角和正攻角的索引
neg_idx = find(original_data_alpha < 0, 1, 'last');
pos_idx = find(original_data_alpha > 0, 1, 'first');

% 获取对应的攻角值
neg_alpha = original_data_alpha(neg_idx);  % 应为-1.666667
pos_alpha = original_data_alpha(pos_idx);  % 应为1.111111

% 创建包含0的攻角数组
data_alpha = [original_data_alpha(1:neg_idx); 0; original_data_alpha(pos_idx:end)];
zero_idx = neg_idx + 1;
num_alpha = length(data_alpha);

fprintf('插入0度攻角\n');
fprintf('负值攻角: %.6f, 正值攻角: %.6f\n', neg_alpha, pos_alpha);
fprintf('攻角数量: %d\n', num_alpha);

data_beta=sort(unique(static_data(:,4)));        %侧滑角数据
num_beta=length(data_beta);               %侧滑角个数
for i=1:num_beta-1
    data_beta1(i)=-data_beta(num_beta-i+1); %负的侧滑角
end
data_beta_real=[data_beta1,data_beta'] ;     %因为之前的侧滑角数目还有用，所以需要重新创建矩阵
num_beta_real=length(data_beta_real);

% 读取系数数据
for i = 1:len
    tmp_alpha = static_data(i,3);
    tmp_beta = static_data(i,4);
    tmp_h = static_data(i,1);
    tmp_ma = static_data(i,2);
    
    % 找到对应的索引位置
    alpha_idx = find(original_data_alpha == tmp_alpha);
    beta_idx = find(data_beta == tmp_beta);
    h_idx = find(data_H/1000 == tmp_h);
    ma_idx = find(data_Ma == tmp_ma);
    
    if ~isempty(alpha_idx) && ~isempty(beta_idx) && ~isempty(h_idx) && ~isempty(ma_idx)
        if tmp_beta == 0  % 处理攻角数据
            % 调整索引以考虑插入的0攻角
            if alpha_idx > neg_idx
                adjusted_idx = alpha_idx + 1;
            else
                adjusted_idx = alpha_idx;
            end
            
            % 填充攻角数据
            Cx(adjusted_idx, ma_idx, h_idx) = static_data(i,5);
            Cy(adjusted_idx, ma_idx, h_idx) = static_data(i,6);
            Cz(adjusted_idx, ma_idx, h_idx) = static_data(i,7);
            Mx(adjusted_idx, ma_idx, h_idx) = static_data(i,8);
            My(adjusted_idx, ma_idx, h_idx) = static_data(i,9);
            Mz(adjusted_idx, ma_idx, h_idx) = static_data(i,10);
        else  % 处理侧滑角数据
            beta_offset = num_alpha;
            Cx(beta_offset + beta_idx, ma_idx, h_idx) = static_data(i,5);
            Cy(beta_offset + beta_idx, ma_idx, h_idx) = static_data(i,6);
            Cz(beta_offset + beta_idx, ma_idx, h_idx) = static_data(i,7);
            Mx(beta_offset + beta_idx, ma_idx, h_idx) = static_data(i,8);
            My(beta_offset + beta_idx, ma_idx, h_idx) = static_data(i,9);
            Mz(beta_offset + beta_idx, ma_idx, h_idx) = static_data(i,10);
        end
    end
end

% 执行0攻角插值
for j = 1:num_Ma
    for k = 1:num_H
        % 计算正确的插值权重
        t = -neg_alpha / (pos_alpha - neg_alpha);  % 应为0.6左右
        
        % 对所有系数进行线性插值
        Cx(zero_idx, j, k) = (1-t) * Cx(neg_idx, j, k) + t * Cx(zero_idx+1, j, k);
        Cy(zero_idx, j, k) = (1-t) * Cy(neg_idx, j, k) + t * Cy(zero_idx+1, j, k);
        Cz(zero_idx, j, k) = (1-t) * Cz(neg_idx, j, k) + t * Cz(zero_idx+1, j, k);
        Mx(zero_idx, j, k) = (1-t) * Mx(neg_idx, j, k) + t * Mx(zero_idx+1, j, k);
        My(zero_idx, j, k) = (1-t) * My(neg_idx, j, k) + t * My(zero_idx+1, j, k);
        Mz(zero_idx, j, k) = (1-t) * Mz(neg_idx, j, k) + t * Mz(zero_idx+1, j, k);
    end
end

% 文件输出部分 - 保持不变
fp = fopen('Aerodynamic.dat','wt');
fprintf(fp,'%s\n','由攻角引起的气动系数');
fprintf(fp,'%s\t\t%s\t%s\n','num_H','num_Ma','num_alpha'); 
fprintf(fp,'%d\t\t%d\t\t%d\n\n',num_H,num_Ma,num_alpha); 
fprintf(fp,'%s\n','data_H');
fprintf(fp,'%g\t',data_H);
fprintf(fp,'\n');
fprintf(fp,'%s\n','data_Ma');
fprintf(fp,'%g\t',data_Ma);
fprintf(fp,'\n');
fprintf(fp,'%s\n','data_alpha');
fprintf(fp,'%g\t',data_alpha);
fprintf(fp,'\n\n');

fprintf(fp,'%s\n','不同攻角下的轴向力Cxa(指向头部为正，行为攻角，列为马赫数，下同）');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        fprintf(fp,'%16.8f\t',Cx(1:num_alpha,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

fprintf(fp,'%s\n','不同攻角下的法向力Cya（向上为正）');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        fprintf(fp,'%16.8f\t',Cy(1:num_alpha,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');
    
fprintf(fp,'%s\n','不同攻角下的俯仰力矩系数Mza');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        fprintf(fp,'%16.8f\t',Mz(1:num_alpha,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

fprintf(fp,'%s\n','由侧滑角引起的气动系数');
fprintf(fp,'%s\n','Num_beta'); 
fprintf(fp,'%d\n',num_beta_real); 
fprintf(fp,'%s\n','data_beta');
fprintf(fp,'%g\t',data_beta_real);
fprintf(fp,'\n\n');

fprintf(fp,'%s\n','不同侧滑角下的轴向力Cxb');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        for k=1:num_beta-1
            fprintf(fp,'%16.8f\t',Cx(num_alpha+num_beta-k,j,i));
        end
        fprintf(fp,'%16.8f\t',Cx(zero_idx,j,i));
        fprintf(fp,'%16.8f\t',Cx(num_alpha+1:num_alpha+num_beta-1,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

fprintf(fp,'%s\n','不同侧滑角下的法向力Cyb');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        for k=1:num_beta-1
            fprintf(fp,'%16.8f\t',Cy(num_alpha+num_beta-k,j,i));
        end
        fprintf(fp,'%16.8f\t',Cy(zero_idx,j,i));
        fprintf(fp,'%16.8f\t',Cy(num_alpha+1:num_alpha+num_beta-1,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

fprintf(fp,'%s\n','不同侧滑角下的侧向力Czb');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        for k=1:num_beta-1
            fprintf(fp,'%16.8f\t',-Cz(num_alpha+num_beta-k,j,i));
        end
        fprintf(fp,'%16.8f\t',Cz(zero_idx,j,i));
        fprintf(fp,'%16.8f\t',Cz(num_alpha+1:num_alpha+num_beta-1,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

fprintf(fp,'%s\n','不同侧滑角下的滚转力矩Mx');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        for k=1:num_beta-1
            fprintf(fp,'%16.8f\t',-Mx(num_alpha+num_beta-k,j,i));
        end
        fprintf(fp,'%16.8f\t',Mx(zero_idx,j,i));
        fprintf(fp,'%16.8f\t',Mx(num_alpha+1:num_alpha+num_beta-1,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

fprintf(fp,'%s\n','不同侧滑角下的偏航力矩My');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        for k=1:num_beta-1
            fprintf(fp,'%16.8f\t',-My(num_alpha+num_beta-k,j,i));
        end
        fprintf(fp,'%16.8f\t',My(zero_idx,j,i));
        fprintf(fp,'%16.8f\t',My(num_alpha+1:num_alpha+num_beta-1,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

fprintf(fp,'%s\n','不同侧滑角下的俯仰力矩Mzb');
for i=1:num_H
    fprintf(fp,'%s%g\n','H',data_H(i)/1000);
    for j=1:num_Ma
        for k=1:num_beta-1
            fprintf(fp,'%16.8f\t',Mz(num_alpha+num_beta-k,j,i));
        end
        fprintf(fp,'%16.8f\t',Mz(zero_idx,j,i));
        fprintf(fp,'%16.8f\t',Mz(num_alpha+1:num_alpha+num_beta-1,j,i));
        fprintf(fp,'\n');
    end
end
fprintf(fp,'\n');

% 处理动导数数据
Mxwx = zeros(1, num_H*num_Ma);
Mywx = zeros(1, num_H*num_Ma);
Mzwx = zeros(1, num_H*num_Ma);
Mxwy = zeros(1, num_H*num_Ma);
Mywy = zeros(1, num_H*num_Ma);
Mzwy = zeros(1, num_H*num_Ma);
Mxwz = zeros(1, num_H*num_Ma);
Mywz = zeros(1, num_H*num_Ma);
Mzwz = zeros(1, num_H*num_Ma);

for i = 1:size(dynamic_data, 1)
    if i <= num_H*num_Ma
        Mxwx(i)=dynamic_data(i,3);
        Mywx(i)=dynamic_data(i,4);
        Mzwx(i)=dynamic_data(i,5);
        Mxwy(i)=dynamic_data(i,6);
        Mywy(i)=dynamic_data(i,7);
        Mzwy(i)=dynamic_data(i,8);
        Mxwz(i)=dynamic_data(i,9);
        Mywz(i)=dynamic_data(i,10);
        Mzwz(i)=dynamic_data(i,11);
    end
end

fprintf(fp,'%s\n','动导数(行为马赫数，列为高度)');
fprintf(fp,'%s\n','mx_wx(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mxwx(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end
    
fprintf(fp,'%s\n','mx_wy(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mxwy(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end

fprintf(fp,'%s\n','mx_wz(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mxwz(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end

fprintf(fp,'%s\n','my_wx(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mywx(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end

fprintf(fp,'%s\n','my_wy(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mywy(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end

fprintf(fp,'%s\n','my_wz(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mywz(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end

fprintf(fp,'%s\n','mz_wx(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mzwx(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end

fprintf(fp,'%s\n','mz_wy(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mzwy(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end

fprintf(fp,'%s\n','mz_wz(对弧度的值，无量纲化l/v)');
for i=1:num_H*num_Ma
    fprintf(fp,'%16.8f\t',Mzwz(i));
    if mod(i,num_Ma)==0
       fprintf(fp,'\n');
    end
end
fclose(fp);

fprintf('数据处理完成，已生成Aerodynamic.dat文件\n');