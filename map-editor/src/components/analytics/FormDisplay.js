import { useState } from "react";
import Chart from "react-google-charts";
import { CircularProgress } from "@material-ui/core";

import { makeStyles } from "@material-ui/core/styles";
import MenuItem from "@material-ui/core/MenuItem";
import FormHelperText from "@material-ui/core/FormHelperText";
import FormControl from "@material-ui/core/FormControl";
import Select from "@material-ui/core/Select";
import OutlinedCard from "./OutlinedCard";

const useStyles = makeStyles((theme) => ({
    formControl: {
        margin: theme.spacing(1),
        minWidth: 300,
        width: "50%",
    },
    selectEmpty: {
        marginTop: theme.spacing(2),
        fontSize: "1em",
        // width: "70%",
    },
}));

export default function FormDisplay({ display, handleChange }) {
    const classes = useStyles();

    const body = (
        <div>
            The most frequently asked question:
            <br />
            What do you like about chicken?
        </div>
    );

    return (
        <div>
            <FormControl className={classes.formControl}>
                <Select
                    value={display}
                    onChange={handleChange}
                    displayEmpty
                    className={classes.selectEmpty}
                    // inputProps={{ "aria-label": "Without label" }}
                >
                    <MenuItem value={"Exhibit Information"}>
                        Exhibit Infomation
                    </MenuItem>
                    <MenuItem value={"Daily Information"}>
                        Daily Infomation
                    </MenuItem>
                    <MenuItem value={"Robot Information"}>
                        Robot Infomation
                    </MenuItem>
                </Select>
            </FormControl>
            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                    flexDirection: "column",
                }}
            >
                <div
                    style={{
                        display: "flex",
                        justifyContent: "space-between",
                    }}
                >
                    <OutlinedCard
                        title="Maximum Number of Interactions"
                        fact={56}
                        subfact="Interactions"
                        body={body}
                    />
                    <OutlinedCard
                        title="Minimum Number of Interactions"
                        fact={0}
                        subfact="Interactions"
                        body={body}
                    />
                </div>
                <div
                    style={{
                        display: "flex",
                        justifyContent: "space-between",
                    }}
                >
                    <OutlinedCard
                        title="Median Number of Interactions"
                        fact={21}
                        subfact="Interactions"
                        body={body}
                    />
                    <OutlinedCard
                        title="Average Number of Interactions"
                        fact={21.81}
                        subfact="Interactions"
                        body={body}
                    />
                </div>
            </div>
        </div>
    );
}
