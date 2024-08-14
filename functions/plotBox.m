function p =plotBox(FH_B,box,color,plotFrame)        
AR_B = FH_B(1:3,1:3);
        %Output the position of the current time step for plotting purposes
        ii =1;
        q(:,ii)  = FH_B(1:3,4);
        Ao_B1{ii} = FH_B(1:3,4);
        R1(:,ii) = FH_B(1:3,1);
        R2(:,ii) = FH_B(1:3,2);
        R3(:,ii) = FH_B(1:3,3);
        if plotFrame
            %Plot the origin of the box with its unit vectors
            tip = [q(:,ii)+ 0.3*R1(:,ii) q(:,ii)+ 0.3*R2(:,ii) q(:,ii)+ 0.3*R3(:,ii)];
            plot3([q(1,ii) tip(1,1)],[q(2,ii) tip(2,1)],[q(3,ii) tip(3,1)],'r'); hold on
            plot3([q(1,ii) tip(1,2)],[q(2,ii) tip(2,2)],[q(3,ii) tip(3,2)],'g');
            plot3([q(1,ii) tip(1,3)],[q(2,ii) tip(2,3)],[q(3,ii) tip(3,3)],'b');
        end
        
        %Create the box
        Ap_1 = AR_B*box.vertices.ds(1,:)' + Ao_B1{ii};
        Ap_2 = AR_B*box.vertices.ds(2,:)' + Ao_B1{ii};
        Ap_3 = AR_B*box.vertices.ds(3,:)' + Ao_B1{ii};
        Ap_4 = AR_B*box.vertices.ds(4,:)' + Ao_B1{ii};
        Ap_5 = AR_B*box.vertices.ds(5,:)' + Ao_B1{ii};
        Ap_6 = AR_B*box.vertices.ds(6,:)' + Ao_B1{ii};
        Ap_7 = AR_B*box.vertices.ds(7,:)' + Ao_B1{ii};
        Ap_8 = AR_B*box.vertices.ds(8,:)' + Ao_B1{ii};
        
        plot3([Ap_1(1) Ap_2(1)],[Ap_1(2) Ap_2(2)],[Ap_1(3) Ap_2(3)],'color',[0.3 0.3 0.3]);%
        plot3([Ap_2(1) Ap_3(1)],[Ap_2(2) Ap_3(2)],[Ap_2(3) Ap_3(3)],'color',[0.3 0.3 0.3]);%
        plot3([Ap_3(1) Ap_4(1)],[Ap_3(2) Ap_4(2)],[Ap_3(3) Ap_4(3)],'color',[0.3 0.3 0.3]);
        plot3([Ap_4(1) Ap_1(1)],[Ap_4(2) Ap_1(2)],[Ap_4(3) Ap_1(3)],'color',[0.3 0.3 0.3]);
        plot3([Ap_5(1) Ap_6(1)],[Ap_5(2) Ap_6(2)],[Ap_5(3) Ap_6(3)],'color',[0.3 0.3 0.3]);%
        plot3([Ap_6(1) Ap_7(1)],[Ap_6(2) Ap_7(2)],[Ap_6(3) Ap_7(3)],'color',[0.3 0.3 0.3]);%
        plot3([Ap_7(1) Ap_8(1)],[Ap_7(2) Ap_8(2)],[Ap_7(3) Ap_8(3)],'color',[0.3 0.3 0.3]);
        plot3([Ap_8(1) Ap_5(1)],[Ap_8(2) Ap_5(2)],[Ap_8(3) Ap_5(3)],'color',[0.3 0.3 0.3]);
        plot3([Ap_1(1) Ap_5(1)],[Ap_1(2) Ap_5(2)],[Ap_1(3) Ap_5(3)],'color',[0.3 0.3 0.3]);
        plot3([Ap_2(1) Ap_6(1)],[Ap_2(2) Ap_6(2)],[Ap_2(3) Ap_6(3)],'color',[0.3 0.3 0.3]);
        plot3([Ap_3(1) Ap_7(1)],[Ap_3(2) Ap_7(2)],[Ap_3(3) Ap_7(3)],'color',[0.3 0.3 0.3]);
        plot3([Ap_4(1) Ap_8(1)],[Ap_4(2) Ap_8(2)],[Ap_4(3) Ap_8(3)],'color',[0.3 0.3 0.3]);
        
        %Color the surfaces of the box
        fill3([Ap_1(1) Ap_4(1) Ap_8(1) Ap_5(1)],[Ap_1(2) Ap_4(2) Ap_8(2) Ap_5(2)],[Ap_1(3) Ap_4(3) Ap_8(3) Ap_5(3)],1,'FaceColor',color,'FaceAlpha',1,'LineWidth',0.01,'EdgeColor',[0.3 0.3 0.3]); hold on%Face D
        fill3([Ap_1(1) Ap_2(1) Ap_6(1) Ap_5(1)],[Ap_1(2) Ap_2(2) Ap_6(2) Ap_5(2)],[Ap_1(3) Ap_2(3) Ap_6(3) Ap_5(3)],1,'FaceColor',color,'FaceAlpha',1,'LineWidth',0.01,'EdgeColor',[0.3 0.3 0.3]);%Face F
        fill3([Ap_1(1) Ap_2(1) Ap_3(1) Ap_4(1)],[Ap_1(2) Ap_2(2) Ap_3(2) Ap_4(2)],[Ap_1(3) Ap_2(3) Ap_3(3) Ap_4(3)],1,'FaceColor',color,'FaceAlpha',1,'LineWidth',0.01,'EdgeColor',[0.3 0.3 0.3]);%Face A
        fill3([Ap_8(1) Ap_7(1) Ap_6(1) Ap_5(1)],[Ap_8(2) Ap_7(2) Ap_6(2) Ap_5(2)],[Ap_8(3) Ap_7(3) Ap_6(3) Ap_5(3)],1,'FaceColor',color,'FaceAlpha',1,'LineWidth',0.01,'EdgeColor',[0.3 0.3 0.3]);%Face C
        fill3([Ap_8(1) Ap_7(1) Ap_3(1) Ap_4(1)],[Ap_8(2) Ap_7(2) Ap_3(2) Ap_4(2)],[Ap_8(3) Ap_7(3) Ap_3(3) Ap_4(3)],1,'FaceColor',color,'FaceAlpha',1,'LineWidth',0.01,'EdgeColor',[0.3 0.3 0.3]);%Face E        
        p = fill3([Ap_2(1) Ap_3(1) Ap_7(1) Ap_6(1)],[Ap_2(2) Ap_3(2) Ap_7(2) Ap_6(2)],[Ap_2(3) Ap_3(3) Ap_7(3) Ap_6(3)],1,'FaceColor',color,'FaceAlpha',1,'LineWidth',0.01,'EdgeColor',[0.3 0.3 0.3]);%Face B
end