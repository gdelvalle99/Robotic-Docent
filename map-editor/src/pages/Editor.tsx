import React, { useState, useEffect } from "react";
import { ExpansionPanel, ExpansionPanelSummary, ExpansionPanelDetails } from '@material-ui/core';

interface Exhibit {
    name: string;
    coordinates: number[];
    description?: string;
    questions?: string[];
    answers?: string[];
    pieces?: {
        name: string;
        coordinates: number[];
        description?: string;
        questions?: string[];
        answers?: string[];
    }
}

export const Editor: React.SFC<Exhibit> = (props) => {

    const [exhibits, setExhibits] = useState<Exhibit[]>([]);

    const getExhibits = async () => {
        let r = await fetch('testdata.json'
            , {
                headers: {
                    'Content-Type': 'application/json',
                    'Accept': 'application/json'
                }
            }
        ).then(function (response) {
            return response.json();
        })
        setExhibits(r);
    }

    useEffect(() => { getExhibits(); }, []);

    return (
        <div className="editor-page">
            <div className="editor-sidebar">
                {exhibits.map(exhibit => (
                    <ExpansionPanel square={false}>
                        <ExpansionPanelSummary>
                            {exhibit.name}
                        </ExpansionPanelSummary>
                        <ExpansionPanelDetails>
                            {exhibit.description}
                        </ExpansionPanelDetails>
                    </ExpansionPanel>
                ))}
            </div>
            <div className="editor-map">
                { /* Put map component here */ }
            </div>
        </div>
    );
}