function [pix,piy,target] = sweepingSearch2(A,x,y)
    [m,n] = size(A);
    persistent cornerHit;
    persistent vx;
    persistent vy;
    persistent startCorner;
    
    if isempty(cornerHit)
        cornerHit = 0;
    end
    
    if isempty(vx)
    vx = 0;
    end
    
    if isempty(vy)
    vy = 0;
    end
    
    if isempty(startCorner)
    startCorner = 0;
    end
    
    corners = [1,1;1,m;n,1;n,m];
    if ~isempty(find(ismember(corners(:,:),[x,y],'rows'),1))
        cornerHit = 1;
    end
    
    if cornerHit == 0
        dist = [corners(:,1)-x,corners(:,2)-y];
        distmin = m * n;
        for i = 1:4
            if norm(dist(i,:)) <= distmin
               pix = corners(i,1);
               piy = corners(i,2);
               startCorner = i;
               distmin = norm(dist(i,:));
            end
        end
        target = [pix,piy];
        pix = x;
        piy = y;
    end
    
    if startCorner == 1 && cornerHit == 0
        vx = 1;
        vy = 1;
    elseif startCorner == 2 && cornerHit == 0
        vx = 1;
        vy = -1;
    elseif startCorner == 3 && cornerHit == 0
        vx = -1;
        vy = 1;    
    elseif startCorner == 4 && cornerHit == 0
        vx = -1;
        vy = -1;
    end
    
    temp_pix = x;
    temp_piy = y;
    
    notValidPosition = 1;
    count = 0;
if cornerHit == 1    
    while notValidPosition
       if temp_piy + vy <= m && temp_piy + vy >= 1
          temp_piy = temp_piy + vy;
       
       elseif temp_pix + vx <= n && temp_pix + vx >= 1
           temp_pix = temp_pix + vx;
           vy = -vy;
       else
           vy = -vy;
           vx = -vx;
           temp_piy = temp_piy + vy;
       end   
       notValidPosition = isnan(A(temp_piy,temp_pix));
       count = count + 1;
    end
        
    if count > 1
        target = [temp_pix,temp_piy];
        pix = x;
        piy = y;
    else
        target = [nan,nan];
        pix = temp_pix;
        piy = temp_piy;
    end
end
end
    