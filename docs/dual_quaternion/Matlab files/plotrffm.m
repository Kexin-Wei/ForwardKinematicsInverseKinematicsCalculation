function plotrffm(startpoint,xvector,zvector,colorpick)
   nx=xvector/norm(xvector);
   nz=zvector/norm(zvector);
   ny=cross(nz,nx);
   
   x_text=startpoint+nx;
   y_text=startpoint+ny;
   z_text=startpoint+nz;
   
   if size(colorpick,2)>2
        r=colorpick(1);
        g=colorpick(2);
        b=colorpick(3);
   else
       r='r';
       g='g';
       b='b';
   end
   
   plotvec3(nx,startpoint,r)
   text(x_text(1),x_text(2),x_text(3),'x')
   xlabel('x');
   ylabel('y');
   zlabel('z');
   hold on
   
   plotvec3(ny,startpoint,g)
   text(y_text(1),y_text(2),y_text(3),'y')
   plotvec3(nz,startpoint,b)
   text(z_text(1),z_text(2),z_text(3),'z')
   
   grid on
end