import React, { useState, useEffect } from "react";
import { ExpansionPanel, ExpansionPanelSummary, ExpansionPanelDetails } from '@material-ui/core';
import { Exhibit } from '../pages/Editor';
import { ExhibitSidebarItem } from '../components/ExhibitSidebarItem';

interface ExhibitProps {
    exhibits: Exhibit[];
}

export const ExhibitSidebar: React.FC<ExhibitProps> = ({ exhibits }: ExhibitProps) => {

    return (
        <div className="exhibit-sidebar-container">
            {exhibits.map(exhibit => {
                return (
                    <ExhibitSidebarItem id={exhibit.id} floor_id={exhibit.floor_id} title={exhibit.title} subtitle={exhibit.subtitle} description={exhibit.description} start_date={exhibit.start_date} end_date={exhibit.end_date} theme={exhibit.theme} questions={exhibit.questions} answers={exhibit.answers} pieces={exhibit.pieces} />
                    );
            })}
        </div>
    );
}