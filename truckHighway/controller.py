import numpy as np

def error_state(veh_state,ref_traj,lookahead=3.0):
        
        x_current = veh_state[0]
        y_current = veh_state[1]
        theta_current = veh_state[2]
        v_current = 1.0
        
        #post process theta
        while theta_current<-np.pi:
            theta_current = theta_current+2*np.pi
        while theta_current>np.pi:
            theta_current = theta_current - 2*np.pi

        dist = np.zeros((1,len(ref_traj[:,1])))
        for i in range(len(ref_traj[:,1])):
            dist[0][i] = dist[0][i] = (x_current+np.cos(theta_current)*lookahead-ref_traj[i][0])**2+(y_current+np.sin(theta_current)*lookahead-ref_traj[i][1])**2
        index = dist.argmin()

        ref_state_current = list(ref_traj[index,:])
        err_theta = 0
        ref = ref_state_current[2]
        act = theta_current

        if( (ref>0 and act>0) or (ref<=0 and act <=0)):
            err_theta = ref-act
        elif( ref<=0 and act > 0):
            if(abs(ref-act)<abs(2*np.pi+ref-act)):
                err_theta = -abs(act-ref)
            else:
                err_theta = abs(2*np.pi + ref- act)
        else:
            if(abs(ref-act)<abs(2*np.pi-ref+act)):
                err_theta = abs(act-ref)
            else: 
                err_theta = -abs(2*np.pi-ref+act)


        RotM = np.array([ 
            [np.cos(-theta_current), -np.sin(-theta_current)],
            [np.sin(-theta_current), np.cos(-theta_current)]
        ])

        errM = np.array([[ref_state_current[0]-x_current],[ref_state_current[1]-y_current]])

        errRM = RotM@errM


        error_state = [errRM[0][0],errRM[1][0],err_theta, 0]

        return error_state


def vir_veh_controller(veh_state,ref_traj,lookahead=3.0):
        
        x_current = veh_state[0]
        y_current = veh_state[1]
        theta_current = veh_state[2]
        v_current = veh_state[3]
        
        #post process theta
        while theta_current<-np.pi:
            theta_current = theta_current+2*np.pi
        while theta_current>np.pi:
            theta_current = theta_current - 2*np.pi

        dist = np.zeros((1,len(ref_traj[:,1])))
        for i in range(len(ref_traj[:,1])):
            dist[0][i] = dist[0][i] = (x_current+np.cos(theta_current)*lookahead-ref_traj[i][0])**2+(y_current+np.sin(theta_current)*lookahead-ref_traj[i][1])**2
        index = dist.argmin()

        ref_state_current = list(ref_traj[index,:])
        err_theta = 0
        ref = ref_state_current[2]
        act = theta_current

        if( (ref>0 and act>0) or (ref<=0 and act <=0)):
            err_theta = ref-act
        elif( ref<=0 and act > 0):
            if(abs(ref-act)<abs(2*np.pi+ref-act)):
                err_theta = -abs(act-ref)
            else:
                err_theta = abs(2*np.pi + ref- act)
        else:
            if(abs(ref-act)<abs(2*np.pi-ref+act)):
                err_theta = abs(act-ref)
            else: 
                err_theta = -abs(2*np.pi-ref+act)


        RotM = np.array([ 
            [np.cos(-theta_current), -np.sin(-theta_current)],
            [np.sin(-theta_current), np.cos(-theta_current)]
        ])

        errM = np.array([[ref_state_current[0]-x_current],[ref_state_current[1]-y_current]])

        errRM = RotM@errM


        error_state = [errRM[0][0],errRM[1][0],err_theta, ref_state_current[3]-v_current]
        
        # pid for steering
        steering = sum([x * y for x, y in zip(error_state, [0.02176878 , 0.72672704 , 0.78409284 ,-0.0105355 ])])
        # pid for throttle
        throttle = sum([x * y for x, y in zip(error_state, [0.37013526 ,0.00507144, 0.15476554 ,1.0235402 ])])
        
        # clip throttle and steering
        throttle = np.clip(throttle, 0, 1)
        steering = np.clip(steering, -1, 1)
        
        return [throttle,steering]