import React, { useState, useEffect } from "react";
import { TourList } from "../components/TourList";
import { newTourLink, demoLink } from "../links";
import axios from "axios";
import { Fab } from "@material-ui/core";
import { Dialog } from "@material-ui/core";
import { DialogActions } from "@material-ui/core";
import { DialogContent } from "@material-ui/core";
import { DialogTitle } from "@material-ui/core";
import { Divider } from "@material-ui/core";
import { TextField } from "@material-ui/core";
import { Button } from "@material-ui/core";
import DeleteIcon from "@material-ui/icons/Delete";
import AddIcon from "@material-ui/icons/Add";
import NavigationIcon from "@material-ui/icons/Navigation";

const convertSimpleISODate = function (dateString) {
    if (dateString) {
        let d = new Date(dateString);

        return d.toISOString().substring(0, 10);
    } else {
        return null;
    }
};

const mockList = [{title: "Tour of France", id: "437829174fdios", subtitle: "this is first tour", start_date: "Mon Apr 26 2021 14:41:56 GMT-0700"},
{title: "Tour of Art", id: "437829174fdios", subtitle: "this is first tour", start_date: "Mon Apr 27 2021 14:41:56 GMT-0700"},
{title: "Tour of Minerals", id: "437829174fdios", subtitle: "this is first tour", start_date: "Mon Apr 25 2021 14:41:56 GMT-0700"},
{title: "Tour of Contemporary Art", id: "437829174fdios", subtitle: "this is first tour", start_date: "Mon Apr 22 2021 14:41:56 GMT-0700"},
{title: "Tour of More Art", id: "437829174fdios", subtitle: "this is first tour", start_date: "Mon Apr 28 2021 14:41:56 GMT-0700"},
{title: "Tour of Test", id: "437829174fdios", subtitle: "this is first tour", start_date: "Mon Apr 28 2021 14:41:56 GMT-0700"}];

export const Tours = () => {
    const [tourList, setTourList] = useState([]);
    const [tour, setTour] = useState({});
    const [openModal, setOpenModal] = useState(false);
    const [openDeleteAlert, setOpenDeleteAlert] = useState(false);

    const handleClick = () => {
        const id = "dbfee438-e5dd-4056-ae2d-4b7a876d351f";
        let r = axios.post(demoLink+id)
            .catch(e=>console.log(e));
    };

    const handleSave = () => {
        // let r = axios.post(newTourLink, tour)
        //     .then(function (response) {
        //         return response.data;
        //     }).then(d => {
        //         setTour({...tour, id: d.id});
        //     }).catch(e=>console.log(e));
        setTourList([...tourList, tour]);
        handleCloseModal();
        setTour({});
    }

    const handleOpenModal = () => {
        setOpenModal(true);
    };

    const handleCloseModal = () => {
        setOpenModal(false);
    };

    const handleCloseAlert = () => {
        setOpenDeleteAlert(false);
    };

    const handleOpenAlert = () => {
        if (tour.title != null && tour.title != "") {
            setOpenDeleteAlert(true);
        }
        else {
            setOpenModal(false);
        }
        
    };

    const handleDeleteClick = () => {
        handleCloseAlert();
        handleCloseModal();
        setTour({});
    };

    const handleChange = (event) => {
        handleAllChanges(tour, {
            [event.target.name]: event.target.value
        });
    }
    
    const handleAllChanges = (tour, diff) => {
        setTour({...tour, ...diff});
    }

    const getTours = () => {
        // let r = axios.get(floorLink).
        //     then(function (response) {
        //         return response;
        //     }).then(item => {
        //         const e = item.data;
        //         setTours(e.tours);
        //     }).catch(e=>console.log(e))

        //DUMMY DATA PLEASE CHANGE LATER
        setTourList(mockList);
    }

    useEffect(() => { 
        getTours(); 
    }, []);

    return (
        <div className="tours-page">
            <TourList tourList={tourList}/>
            <div className="tours-buttons">
                <Fab
                    variant="extended"
                    size="large"
                    onClick={handleOpenModal}
                >
                    <AddIcon />
                    Create New Tour
                </Fab>
                <Fab
                    variant="extended"
                    size="large"
                    onClick={handleClick}
                >
                    <NavigationIcon />
                    Start Tour
                </Fab>
            </div>
            <Dialog
                open={openModal}
                onClose={handleCloseModal}
                fullWidth={true}
            >
                <DialogTitle className="tour-item-title-container">Add a New Tour</DialogTitle>
                <DialogContent>
                    <div className="tour-item-content-container">
                        <TextField
                            className="tour-item-title"
                            label="Title"
                            variant="outlined"
                            name="title"
                            onBlur={handleChange}
                            defaultValue=" "
                            fullWidth
                        />
                        <TextField
                            className="tour-item-subtitle"
                            label="Subtitle"
                            variant="outlined"
                            name="subtitle"
                            onBlur={handleChange}
                            defaultValue=" "
                            fullWidth
                        />
                        <TextField
                                className="tour-item-startdate"
                                label="Start Date"
                                variant="outlined"
                                type="date"
                                name="start_date"
                                onBlur={handleChange}
                                defaultValue={convertSimpleISODate(new Date())}
                                InputLabelProps={{ shrink: true }}
                                fullWidth
                            />
                        <TextField
                            className="tour-item-starttime"
                            label="Start Time"
                            variant="outlined"
                            type="time"
                            name="start_time"
                            onBlur={handleChange}
                            defaultValue={convertSimpleISODate(new Date())}
                            InputLabelProps={{ shrink: true }}
                            fullWidth
                        />
                    </div>
                </DialogContent>
                <DialogActions>
                    <div className="tour-buttons">
                        <div className="tour-button-cancel">
                            <Button onClick={handleOpenAlert} variant="outlined">
                                Cancel
                            </Button>
                        </div>
                        <div className="tour-button-add">
                            <Button onClick={handleSave} variant="outlined">
                                Add
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
            <Dialog open={openDeleteAlert} onClose={handleCloseAlert}>
                <DialogTitle>Delete this tour?</DialogTitle>
                <DialogActions>
                    <div className="tour-delete-alert-buttons">
                        <div className="tour-delete-alert-button-delete">
                            <Button
                                onClick={handleDeleteClick}
                                variant="outlined"
                            >
                                Delete
                            </Button>
                        </div>
                        <div className="tour-delete-alert-button-cancel">
                            <Button
                                onClick={handleCloseAlert}
                                variant="outlined"
                            >
                                Cancel
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
        </div>
    );
};
