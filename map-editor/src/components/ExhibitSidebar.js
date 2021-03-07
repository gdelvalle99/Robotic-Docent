import React, { useState, useEffect } from "react";
import { ExhibitSidebarItem } from '../components/ExhibitSidebarItem';

export const ExhibitSidebar = (props) => {

    return (
        <div className="exhibit-sidebar-container">
            {props.exhibitsList.exhibits && props.exhibitsList.exhibits.length > 0 
                ?   props.exhibitsList.exhibits.map(exhibit => {
                        return (
                            <ExhibitSidebarItem key={exhibit.id} id={exhibit.id} floor_id={exhibit.floor_id} title={exhibit.title} subtitle={exhibit.subtitle} description={exhibit.description} start_date={exhibit.start_date} end_date={exhibit.end_date} theme={exhibit.theme} questions={exhibit.questions} answers={exhibit.answers} />
                            );
                    })
                :   <p>Empty</p>
            }
        </div>
    );
}