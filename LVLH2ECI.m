function [r_out_ECI, v_out_ECI] = LVLH2ECI(C,Cdot,r_LVLH,r_ref,v_LVLH,v_ref)
[m,n] = size(r_LVLH);

for i = 1:m
    r_out_ECI_tilda = (C')*((r_LVLH(i,:))'); 
    r_out_ECI(i,:) = (r_out_ECI_tilda + (r_ref(i,:))')';

    v_out_ECI_tilda = (C')*((v_LVLH(i,:))' - Cdot*r_out_ECI_tilda); 
    v_out_ECI(i,:) = (v_out_ECI_tilda + (v_ref(i,:))')';
    
end 