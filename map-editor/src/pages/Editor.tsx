import React from "react";
import { ExpansionPanel, ExpansionPanelSummary, ExpansionPanelDetails } from '@material-ui/core';


export default function Editor() {
    return (
        <div className="editor-page">
            <div className="editor-sidebar">
                <ExpansionPanel square={false} className="editor-sidebar-dropdown">
                    <ExpansionPanelSummary>
                        Title
                    </ExpansionPanelSummary>
                    <ExpansionPanelDetails className="dropdown-open">
                        Lorem  ipsum
                    </ExpansionPanelDetails>
                </ExpansionPanel>
            </div>
            <div className="editor-map">
                <div>Hi there</div>
                <div>Hi there</div>
                <div>Hi there</div>
                <div>Hi there</div>
                <div>Hi there</div>
                <div>Hi there</div>
            </div>
        </div>
    );

}