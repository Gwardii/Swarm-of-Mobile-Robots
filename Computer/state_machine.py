from states import *

class StateMachine(object):
    def __init__(self,character) -> None:
        self.character=character
        self.states={}
        self.transitions={}
        self.current_state:State=None
        self.current_transition:Transition=None
        self.previous_state=self.current_state
    
    def Set_state(self,state_name)->None:
        self.current_state=self.states[state_name]
    
    def Transition(self,transition_name)->None:
        self.current_transition=self.transitions[transition_name]

    def Execute(self):
        if(self.current_transition):
            # self.current_transition.Execute()
            self.Set_state(self.current_transition.to_state)
            self.current_transition=None
        self.current_state.Execute()
    