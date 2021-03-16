import React, { useState, useEffect } from "react";
import { ExhibitSidebarModal } from './ExhibitSidebarModal';
import { Button } from '@material-ui/core';
import { createStyles, makeStyles } from '@material-ui/core/styles';
import AddCircleOutlineIcon from '@material-ui/icons/AddCircleOutline';

const useStyles = makeStyles((theme) =>
    createStyles({
        buttonSidebarStyle: {
            marginTop: 0,
            margin: 10,
            border: '2px dashed #EBEBEB'
        }
    }),
);

export const ExhibitSidebar = (props) => {
    const [exhibitList, setExhibitList] = useState([]);
    const classes = useStyles();

    const addExhibit = () => {
        setExhibitList(exhibitList => [...exhibitList, <ExhibitSidebarModal/>])
    }

    const setInitialExhibitList = () => {
        if (props.exhibitListContainer.exhibits != null) {
            setExhibitList(props.exhibitListContainer.exhibits);
        }  
    }

    useEffect(() => { 
        setInitialExhibitList();
    }, [props]);

    return (
        <div className="exhibit-sidebar-container">
            {exhibitList && exhibitList.length > 0 && (
                    exhibitList.map(exhibit => {
                            return (
                                <ExhibitSidebarModal key={exhibit.id} id={exhibit.id} floor_id={exhibit.floor_id} title={exhibit.title} subtitle={exhibit.subtitle} description={exhibit.description} start_date={exhibit.start_date} end_date={exhibit.end_date} theme={exhibit.theme} questions={exhibit.questions} answers={exhibit.answers} />
                                );
                        })
                )
            }
            <div className="exhibit-sidebar-add-button">
                <Button className={classes.buttonSidebarStyle} onClick={addExhibit}>
                    <AddCircleOutlineIcon fontSize='large' />
                </Button>
            </div>
        </div>
    );
}