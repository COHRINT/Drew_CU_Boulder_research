function detection = takeMeasurement(alpha,beta,xtx,xty,x,y)
    r = rand;
    if (xtx ~= x || xty ~= y)
        if (r<alpha)
            detection = 1;
        else
            detection = 0;
        end
    else
        if (r<beta)
            detection = 0;
        else
            detection = 1;
        end
    end
end