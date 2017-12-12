package org.firstinspires.ftc;

/**
 * Created by Sarthak on 12/2/2017.
 */

public class Rule {
    private String startState;
    private boolean condition;
    private String action;
    private String endState;

    public Rule(String startState, boolean condition, String action, String endState){
        this.startState = startState;
        this.condition = condition;
        this.action = action;
        this.endState = endState;
    }

    public String getStartState() {
        return startState;
    }

    public boolean isCondition() {
        return condition;
    }

    public String getAction() {
        return action;
    }

    public String getEndState() {
        return endState;
    }
}
