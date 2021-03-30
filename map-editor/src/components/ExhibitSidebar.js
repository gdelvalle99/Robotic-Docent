import React, { useState, useEffect } from "react";
import { ExhibitSidebarModal } from './ExhibitSidebarModal';
import { Button } from '@material-ui/core';
import { createStyles, makeStyles } from '@material-ui/core/styles';
import { deleteExhibitLink } from '../links';
import AddCircleOutlineIcon from '@material-ui/icons/AddCircleOutline';
import { newExhibitLink } from '../links';
import { existingExhibitLink } from '../links';
import { floor_id } from '../links';
import axios from 'axios';

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

    //Handles save and sending post request for new exhibit to add to the floor
    const handleSaveNewExhibit = (exhibit) => {
        let data = {...exhibit, floor_id: floor_id}
        let r = axios.post(newExhibitLink, data)
            .then(function (response) {
                return response.data;
            }).then(d => {
                const updatedExhibit = {...data, id: d.id};
                let updatedList = exhibitList.map(item => {
                   if (item.id === undefined) {
                       return updatedExhibit;
                   }
                   return item
                });
                setExhibitList(updatedList);
                return d.id;
            }).catch(e=>console.log(e))
        return r;
    }

    //Handles save and sending post request for existing exhibit on the floor
    const handleSaveExistingExhibit = (exhibit) => {
        let r = axios.post(existingExhibitLink, exhibit)
            .catch(e=>console.log(e))
    }

    //Handles delete and sending get request for existing exhibit on the floor
    const handleDeleteExhibit = (id) => {
        if (id !== undefined) {
            let link = deleteExhibitLink + id;
            let r = axios.get(link)
                .catch(e=>console.log(e))
            const updatedList = exhibitList.filter(item => item.id !== id);
            setExhibitList(updatedList);
        }
        else {
            const updatedList = exhibitList.filter(item => item.id !== undefined);
            setExhibitList(updatedList);
        }
    }

    const addExhibit = () => {
        const emptyExhibit = {}
        setExhibitList(exhibitList => [...exhibitList, emptyExhibit])
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
                                <ExhibitSidebarModal key={exhibit.id} exhibit={exhibit} handleDeleteExhibit={handleDeleteExhibit} handleSaveNewExhibit={handleSaveNewExhibit} handleSaveExistingExhibit={handleSaveExistingExhibit} />
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