// Function to handle the interrupt
void FL_Stall_Interrupt() {
    FL_Stall_State = true;
}

void FR_Stall_Interrupt() {
    FR_Stall_State = true;
}

void RL_Stall_Interrupt() {
    RL_Stall_State = true;
}

void RR_Stall_Interrupt() {
    RR_Stall_State = true;
}

